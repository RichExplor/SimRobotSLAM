"""Read and apply the existing Nav2 speed limits while the patrol is reserved."""

import math
import threading
import time
from copy import deepcopy

from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter, parameter_value_to_python
from std_srvs.srv import SetBool


class NavigationSettings:
    FIELDS = ("max_forward_speed", "max_lateral_speed", "max_angular_speed")
    PARAMETERS = {
        "/controller_server": ("FollowPath.vx_max", "FollowPath.vx_min", "FollowPath.vy_max",
                               "FollowPath.wz_max", "FollowPath.motion_model",
                               "FollowPath.rotate_to_heading_angular_vel"),
        "/velocity_smoother": ("max_velocity", "min_velocity"),
    }

    def __init__(self, node):
        self.node = node
        self.clients = {
            name: (node.create_client(GetParameters, name + "/get_parameters"),
                   node.create_client(SetParametersAtomically, name + "/set_parameters_atomically"),
                   node.create_client(GetState, name + "/get_state"))
            for name in self.PARAMETERS
        }
        self._mutex = threading.Lock()
        self._transaction = threading.Lock()
        self._values = None
        self._updated = 0.0
        self._last_poll = 0.0
        self._reading = False
        self.applying = False
        self.reason = "等待执行端参数"
        self._abort = False

    def snapshot(self):
        with self._mutex:
            values = deepcopy(self._values)
            fresh = time.monotonic() - self._updated < 12
            ready = all(client.service_is_ready() for clients in self.clients.values()
                        for client in clients)
            return {"available": bool(values and fresh and ready and not self.applying),
                    "values": values if fresh else None, "applying": self.applying,
                    "reason": self.reason if fresh or self.reason else "执行端参数已过期",
                    "scope": "当前 Nav2 进程；导航重启后恢复 Launch/YAML 配置"}

    def poll(self):
        if self.applying or self._reading or time.monotonic() - self._last_poll < 3:
            return
        self._last_poll = time.monotonic()
        self._reading = True
        threading.Thread(target=self._refresh, daemon=True).start()

    def _call(self, client, request, job=None, mutation=False):
        if not client.service_is_ready():
            raise ValueError(f"接口未就绪：{client.srv_name}")
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(4):
            if not mutation:
                raise ValueError(f"读取超时：{client.srv_name}")
            self._abort = True
            self.reason = f"下发结果未确认：{client.srv_name}；执行器保持锁定"
            self.node._reply(job, {"pending": True, "detail": self.reason}, None)
            while self.node.context.ok() and not done.wait(.2):
                pass
            if not done.is_set():
                raise ValueError(self.reason)
        return future.result()

    def _read(self):
        result = {}
        for name, names in self.PARAMETERS.items():
            get, _, lifecycle = self.clients[name]
            state = self._call(lifecycle, GetState.Request()).current_state
            if state.id != 3:
                raise ValueError(f"{name} 未激活：{state.label}")
            request = GetParameters.Request()
            request.names = list(names)
            response = self._call(get, request)
            if len(response.values) != len(names) or any(value.type == 0 for value in response.values):
                raise ValueError(f"{name} 缺少运动限制参数，当前控制器不受支持")
            result[name] = {key: parameter_value_to_python(value)
                            for key, value in zip(names, response.values)}
        return result

    def _document(self, parameters):
        controller, smoother = (parameters[name] for name in self.PARAMETERS)
        maximum, minimum = smoother["max_velocity"], smoother["min_velocity"]
        if len(maximum) != 3 or len(minimum) != 3:
            raise ValueError("速度平滑器参数必须包含三个方向")
        for name, values in parameters.items():
            for key, value in values.items():
                if key == "FollowPath.motion_model":
                    continue
                numbers = value if isinstance(value, list) else [value]
                if any(isinstance(number, bool) or not isinstance(number, (int, float))
                       or not math.isfinite(number) for number in numbers):
                    raise ValueError(f"{name}/{key} 参数无效")
        lateral = controller["FollowPath.motion_model"] == "Omni"
        return {"max_forward_speed": min(controller["FollowPath.vx_max"], maximum[0]),
                "max_lateral_speed": min(controller["FollowPath.vy_max"], maximum[1]) if lateral else 0.,
                "max_angular_speed": min(controller["FollowPath.wz_max"], maximum[2]),
                "lateral_available": lateral, "motion_model": controller["FollowPath.motion_model"]}

    def _store(self, parameters):
        values = self._document(parameters)
        with self._mutex:
            self._values = values
            self._updated = time.monotonic()
            self.reason = ""

    def _refresh(self):
        try:
            with self._transaction:
                self._store(self._read())
        except (ValueError, TypeError, RuntimeError) as error:
            with self._mutex:
                self._updated = 0.
                self.reason = str(error)
        finally:
            self._reading = False

    def start(self, job, snapshot):
        payload = job["payload"]
        if set(payload) != set(self.FIELDS):
            raise ValueError("仅支持纵向、横向和角速度限制")
        limits = ((.05, 2.), (0., 1.), (.05, 2.))
        for key, (low, high) in zip(self.FIELDS, limits):
            value = payload[key]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not low <= value <= high):
                raise ValueError(f"{key} 必须在 {low}～{high} 之间")
        if self.applying or self.node._preview or self.node._tasks.operation:
            raise ValueError("机器人正在处理其他命令")
        patrol = snapshot.get("patrol") or {}
        velocity = snapshot.get("velocity") or {}
        if not patrol.get("fresh") or patrol.get("state") != "idle":
            raise ValueError("仅可在巡检执行器空闲时下发参数")
        if not velocity.get("fresh") or not velocity.get("stopped"):
            raise ValueError("轮速未就绪或机器人尚未停稳")
        self.applying = True
        self._abort = False
        self.reason = "正在下发并核对执行端参数"
        threading.Thread(target=self._apply, args=(job,), daemon=True).start()

    def _set(self, name, values, job):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(key, value=value).to_parameter_msg()
                              for key, value in values.items()]
        response = self._call(self.clients[name][1], request, job, mutation=True)
        if not response.result.successful:
            raise ValueError(f"{name} 拒绝参数：{response.result.reason}")

    @staticmethod
    def _verify(actual, expected):
        if actual != expected:
            raise ValueError("执行端参数回读与下发值不一致")

    def _apply(self, job):
        locked, safe, changed, originals = False, True, [], None
        result, error = None, None
        with self._transaction:
            try:
                if job["abandoned"].is_set():
                    raise ValueError("请求已过期，请读取当前参数后重试")
                snapshot = self.node.snapshot()
                patrol, velocity = snapshot.get("patrol") or {}, snapshot.get("velocity") or {}
                if not patrol.get("fresh") or patrol.get("state") != "idle":
                    raise ValueError("巡检执行器不再空闲，已中止下发")
                if not velocity.get("fresh") or not velocity.get("stopped"):
                    raise ValueError("机器人未停稳，已中止下发")
                request = SetBool.Request()
                request.data = True
                response = self._call(self.node._patrol_lock, request, job, mutation=True)
                if not response.success:
                    raise ValueError(response.message)
                locked = True
                if self._abort:
                    raise ValueError("执行器锁定响应超时，已中止参数下发")
                originals = self._read()
                current = self._document(originals)
                values = job["payload"]
                if not current["lateral_available"] and values["max_lateral_speed"] != 0:
                    raise ValueError("当前导航模型不允许横向移动，横向速度必须为 0")
                desired = deepcopy(originals)
                controller = desired["/controller_server"]
                controller.update({"FollowPath.vx_max": float(values["max_forward_speed"]),
                                   "FollowPath.vx_min": max(controller["FollowPath.vx_min"], -values["max_forward_speed"]),
                                   "FollowPath.vy_max": float(values["max_lateral_speed"]),
                                   "FollowPath.wz_max": float(values["max_angular_speed"]),
                                   "FollowPath.rotate_to_heading_angular_vel": min(
                                       controller["FollowPath.rotate_to_heading_angular_vel"], values["max_angular_speed"])})
                smoother = desired["/velocity_smoother"]
                smoother["max_velocity"] = [float(values[key]) for key in self.FIELDS]
                smoother["min_velocity"] = [max(old, -float(values[key]))
                                            for old, key in zip(smoother["min_velocity"], self.FIELDS)]
                for name, parameters in desired.items():
                    # motion_model is read only here; changing it needs a separate navigation deployment.
                    updated = {key: value for key, value in parameters.items() if value != originals[name][key]}
                    if updated:
                        changed.append((name, list(updated)))
                        self._set(name, updated, job)
                    if self._abort:
                        raise ValueError("参数下发超时，恢复原参数")
                actual = self._read()
                self._verify(actual, desired)
                self._store(actual)
                result = {"applied": True, "settings": self.snapshot()}
            except Exception as failure:
                error = str(failure)
                try:
                    for name, keys in reversed(changed):
                        self._set(name, {key: originals[name][key] for key in keys}, job)
                    if changed:
                        actual = self._read()
                        self._verify(actual, originals)
                        self._store(actual)
                        error += "；已确认恢复原参数"
                except Exception as rollback_error:
                    safe = False
                    error += f"；恢复未确认，执行器保持锁定：{rollback_error}"
                self.node.get_logger().error(error)
            finally:
                if locked and safe:
                    try:
                        request = SetBool.Request()
                        request.data = False
                        response = self._call(self.node._patrol_lock, request, job, mutation=True)
                        if not response.success:
                            raise ValueError(response.message)
                        locked = False
                    except Exception as unlock_error:
                        error = f"释放执行器锁未确认：{unlock_error}"
                self.applying = locked
                self.reason = error or ""
                if result and not error:
                    result["settings"] = self.snapshot()
                self.node._reply(job, result, error)
