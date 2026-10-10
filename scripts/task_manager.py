"""ROS-side task dispatch, durable results and ownership checks for Web commands."""

import json
import sqlite3
import time
import uuid

from action_msgs.srv import CancelGoal
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import SetBool, Trigger
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from task_journal import TERMINAL, TaskJournal, utc_now


class TaskManager:
    def __init__(self, node, directory):
        self.node = node
        self.journal = TaskJournal(directory) if directory else None
        self.operation = None
        self.last_sync = 0.0
        self.last_signature = None
        self.last_sample = 0.0
        self.parameter_client = node.create_client(SetParametersAtomically, "/route_patrol/set_parameters_atomically")
        self.clients = {name: node.create_client(Trigger, "/route_patrol/" + name)
                        for name in ("start_web", "pause", "resume", "cancel", "finish_web")}
        self.cancel_clients = {}
        self.odom = None
        self.odom_wall = 0.0
        node.create_subscription(Odometry, "/wheel/odometry", self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, message):
        self.odom = message
        self.odom_wall = time.monotonic()

    def available(self):
        return not self.unavailable_reasons()

    def unavailable_reasons(self):
        missing = []
        if not self.journal:
            missing.append("未配置 task_data_dir（任务记录目录）")
        if not self.parameter_client.service_is_ready():
            missing.append("/route_patrol/set_parameters_atomically")
        missing.extend("/route_patrol/" + name for name, client in self.clients.items()
                       if not client.service_is_ready())
        return missing

    def list(self):
        if not self.journal:
            return []
        keys = {"id", "status", "gateway_revision", "updated_at", "restored"}
        return [{key: document.get(key) for key in keys} for document in self.journal.list()]

    def get(self, task_id):
        if not self.journal:
            raise KeyError(task_id)
        return self.journal.get(task_id)

    def poll(self, snapshot):
        if not self.journal:
            return
        patrol = snapshot.get("patrol") or {}
        if patrol.get("fresh") and patrol.get("task_id"):
            try:
                document = self.journal.get(patrol["task_id"], details=False)
            except KeyError:
                document = None
            if document is not None:
                if (document.get("cancel_requested") and document["status"] not in TERMINAL
                        and patrol["state"] not in {"cancelling", "stop_unconfirmed", "finished"}
                        and time.monotonic() - getattr(self, "last_cancel_retry", 0) >= 1):
                    self.clients["cancel"].call_async(Trigger.Request())
                    self.last_cancel_retry = time.monotonic()
                state = patrol["state"]
                outcome = patrol.get("outcome")
                status = (outcome if outcome in TERMINAL else
                          "paused" if state == "paused" else "pausing" if state == "pausing" else
                          "cancelling" if state in {"cancelling", "stop_unconfirmed"} else "running")
                signature = (patrol["task_id"], state, outcome, patrol.get("completed_points"),
                             patrol.get("action_index"), patrol.get("event_seq", 0),
                             json.dumps(patrol.get("active_goals", [])))
                sample = (snapshot.get("pose") if snapshot.get("ready") and state not in {"finished", "idle"}
                          and time.monotonic() - self.last_sample >= .5 else None)
                if (signature != self.last_signature or sample or
                        (document["status"] not in TERMINAL and time.monotonic() - self.last_sync >= 1)):
                    values = {"status": status, "completed_points": patrol.get("completed_points", 0),
                              "state": state, "current_node": patrol.get("station_node_id"),
                              "current_point_name": patrol.get("station_name"),
                              "action_index": patrol.get("action_index"), "action": patrol.get("action"),
                              "executor_instance": patrol.get("executor_instance"),
                              "active_goals": patrol.get("active_goals", []),
                              "detail": patrol.get("halt_detail") or patrol.get("last_summary", {}).get("detail", ""),
                              "event_seq": patrol.get("event_seq", 0)}
                    self.journal.update(document["id"], values, patrol.get("events", []), sample)
                    self.last_signature, self.last_sync = signature, time.monotonic()
                    if sample:
                        self.last_sample = time.monotonic()
        if self.operation:
            pending = self.operation
            if (pending["stage"] == "starting" and patrol.get("fresh")
                    and patrol.get("task_id") == pending["task_id"]
                    and patrol["state"] not in {"idle", "previewing"}):
                self._started(pending)
                return
            if pending["stage"] == "interrupting":
                self._poll_interrupt(pending)
                return
            if time.monotonic() > pending["deadline"]:
                if pending["stage"] in {"starting", "locking", "loading"}:
                    # A service timeout does not prove that the robot rejected the task.
                    self.node._reply(pending["job"], {"task": self.get(pending["task_id"]), "pending": True}, None)
                    pending["deadline"] = time.monotonic() + 20
                elif pending["stage"] in {"restoring", "releasing"}:
                    self.journal.update(pending["task_id"], {"detail": "路线图恢复未确认，执行器保持锁定"})
                    if pending.get("job"):
                        self.node._reply(pending["job"], None, "路线图恢复未确认，需检查导航和巡检节点")
                    self.operation = None
                else:
                    self._fail(pending, "任务下发步骤超时")
            return
        if self.node._preview:
            return
        if not patrol.get("fresh"):
            for document in self.journal.list(pending_only=True):
                if document["status"] not in TERMINAL:
                    self._interrupt(document, patrol)
                    break
            return
        for document in self.journal.list(pending_only=True):
            if document.get("restored", True):
                continue
            if patrol.get("task_id") == document["id"] and patrol["state"] == "finished":
                self._restore(document, "finish_web")
                break
            if (patrol["state"] == "idle" and patrol.get("task_id") == document["id"]
                    and document["status"] in TERMINAL and not document.get("active_goals")):
                self._restore(document, "none")
                break
            if patrol["state"] in {"idle", "previewing"} and patrol.get("task_id") != document["id"]:
                self._interrupt(document, patrol)
                break

    def process(self, job, snapshot):
        if not self.journal:
            self.node._reply(job, None, "需配置 task_data_dir 才能下发任务")
            return
        try:
            kind = job["kind"]
            if kind == "start":
                self._start(job, snapshot)
            elif kind == "delete":
                self.journal.delete(job["payload"]["task_id"])
                self.node._reply(job, {"deleted": True}, None)
            else:
                self._control(job, snapshot)
        except (ValueError, KeyError, TypeError, OSError, sqlite3.Error) as error:
            self.node._reply(job, None, str(error))

    def _start(self, job, snapshot):
        payload = job["payload"]
        task = payload["task"]
        task_id = str(uuid.UUID(task["id"]))
        try:
            old = self.journal.get(task_id)
        except KeyError:
            old = None
        if old is not None:
            if old["route"] != payload["route"]:
                raise ValueError("任务 ID 已用于其他配置")
            self.node._reply(job, {"task": old}, None)
            return
        if self.operation or self.node._preview:
            raise ValueError("机器人正在处理其他命令")
        if not self.available() or not snapshot.get("ready") or not snapshot.get("preview_available"):
            raise ValueError(snapshot.get("reason") or snapshot.get("preview_reason")
                             or snapshot.get("task_reason") or "任务或导航服务未就绪")
        loaded = snapshot["loaded_map"]
        route = payload["route"]
        if (route["map_id"], route["version_id"], payload["fingerprint"]) != (
                loaded["map_id"], loaded["version_id"], loaded["fingerprint"]):
            raise ValueError("任务地图与机器人实际加载版本不一致")
        patrol = snapshot.get("patrol") or {}
        if not patrol.get("fresh") or patrol["state"] != "idle":
            raise ValueError("巡检执行器未空闲")
        document, _ = self.journal.create({**task, "fingerprint": payload["fingerprint"],
                                           "executor_instance": patrol.get("executor_instance")})
        directory = self.journal.directory / task_id
        directory.mkdir(exist_ok=True)
        filename = directory / "route.geojson"
        filename.write_text(json.dumps(payload["graph"], allow_nan=False), encoding="utf-8")
        pending = {"task_id": task_id, "job": job, "payload": payload, "stage": "locking",
                   "deadline": time.monotonic() + 15, "locked": False, "original": None,
                   "filename": str(filename)}
        self.operation = pending
        request = SetBool.Request()
        request.data = True
        self.node._patrol_lock.call_async(request).add_done_callback(lambda future: self._on_lock(future, pending))

    def _on_lock(self, future, pending):
        try:
            response = future.result()
        except Exception as error:
            self.node.get_logger().warning(f"执行器锁定回复未确认，保持请求待确认：{error}")
            pending["deadline"] = time.monotonic() + 20
            return
        try:
            if self.operation is not pending:
                if response.success:
                    request = SetBool.Request()
                    request.data = False
                    self.node._patrol_lock.call_async(request)
                return
            if not response.success:
                raise ValueError(response.message)
            pending["locked"] = True
            pending["stage"] = "parameters"
            from rcl_interfaces.srv import GetParameters
            request = GetParameters.Request()
            request.names = ["graph_filepath"]
            self.node._graph_parameters.call_async(request).add_done_callback(lambda result: self._on_parameters(result, pending))
        except Exception as error:
            if self.operation is pending:
                self._fail(pending, str(error))

    def _on_parameters(self, future, pending):
        if self.operation is not pending or pending["stage"] != "parameters":
            return
        try:
            values = future.result().values
            original = values[0].string_value if values else ""
            if not original:
                raise ValueError("导航没有配置可恢复的原路线图")
            pending["original"] = original
            self.journal.update(pending["task_id"], {"original_graph": original})
            pending["stage"] = "loading"
            from nav2_msgs.srv import SetRouteGraph
            request = SetRouteGraph.Request()
            request.graph_filepath = pending["filename"]
            self.node._set_graph.call_async(request).add_done_callback(lambda result: self._on_graph(result, pending))
        except Exception as error:
            self._fail(pending, str(error))

    def _on_graph(self, future, pending):
        if self.operation is not pending or pending["stage"] != "loading":
            return
        try:
            response = future.result()
        except Exception as error:
            self.node.get_logger().warning(f"路线图加载回复未确认，保持执行器锁定：{error}")
            pending["deadline"] = time.monotonic() + 20
            return
        try:
            if not response.success:
                raise ValueError("Route Server 拒绝任务路线图")
            payload = pending["payload"]
            stations = [{**point, "node_id": node_id} for point, node_id in
                        zip(payload["route"]["points"], payload["station_ids"])]
            mission = {"task_id": pending["task_id"], "map_id": payload["route"]["map_id"],
                       "version_id": payload["route"]["version_id"], "fingerprint": payload["fingerprint"],
                       "stations": stations, "journal_dir": str(self.journal.directory)}
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter(name="web_mission", value=ParameterValue(
                type=4, string_value=json.dumps(mission, allow_nan=False)))]
            pending["stage"] = "configuring"
            self.parameter_client.call_async(request).add_done_callback(lambda result: self._on_configured(result, pending))
        except Exception as error:
            self._fail(pending, str(error))

    def _on_configured(self, future, pending):
        if self.operation is not pending or pending["stage"] != "configuring":
            return
        try:
            result = future.result().result
            if not result.successful:
                raise ValueError(result.reason)
            if pending.get("cancel_requested"):
                self._fail(pending, "已撤销下发请求，未开始运动")
                return
            pending["stage"] = "starting"
            self.clients["start_web"].call_async(Trigger.Request()).add_done_callback(lambda response: self._on_started(response, pending))
        except Exception as error:
            self._fail(pending, str(error))

    def _on_started(self, future, pending):
        if self.operation is not pending:
            return
        try:
            response = future.result()
        except Exception as error:
            # Losing a service reply does not prove that start_web was rejected.
            self.node.get_logger().warning(f"任务启动回复未确认，等待实际执行状态：{error}")
            pending["deadline"] = time.monotonic() + 20
            return
        try:
            if not response.success:
                raise ValueError(response.message)
            self._started(pending)
        except Exception as error:
            self._fail(pending, str(error))

    def _started(self, pending):
        self.node._reply(pending["job"], {"task": self.get(pending["task_id"]), "accepted": True}, None)
        self.operation = None
        if pending.get("cancel_requested"):
            self.clients["cancel"].call_async(Trigger.Request())

    def _fail(self, pending, detail):
        if self.operation is not pending:
            return
        self.journal.update(pending["task_id"], {
            "status": "cancelled" if pending.get("cancel_requested") else "failed", "detail": detail},
            [{"seq": -4, "kind": "dispatch_rejected", "detail": detail}])
        pending["error"] = detail
        self._restore(self.journal.get(pending["task_id"], details=False), "unlock", pending)

    def _restore(self, document, release, pending=None):
        from nav2_msgs.srv import SetRouteGraph
        pending = pending or {"task_id": document["id"], "job": None, "locked": release == "unlock"}
        pending.update(stage="restoring", release=release, deadline=time.monotonic() + 5)
        self.operation = pending
        original = document.get("original_graph")
        if not original:
            self._release(pending)
            return
        request = SetRouteGraph.Request()
        request.graph_filepath = original
        self.node._set_graph.call_async(request).add_done_callback(lambda future: self._on_restored(future, pending))

    def _on_restored(self, future, pending):
        if self.operation is not pending:
            return
        try:
            if not future.result().success:
                raise ValueError("原路线图恢复失败")
            self._release(pending)
        except Exception as error:
            self.journal.update(pending["task_id"], {"detail": f"{error}；保持执行器锁定"})
            if pending.get("job"):
                self.node._reply(pending["job"], None, str(error))
            self.operation = None

    def _release(self, pending):
        pending["stage"] = "releasing"
        if pending["release"] == "finish_web":
            future = self.clients["finish_web"].call_async(Trigger.Request())
        elif pending.get("locked"):
            request = SetBool.Request()
            request.data = False
            future = self.node._patrol_lock.call_async(request)
        else:
            self._released(pending)
            return
        future.add_done_callback(lambda response: self._on_released(response, pending))

    def _on_released(self, future, pending):
        if self.operation is not pending:
            return
        try:
            if not future.result().success:
                raise ValueError("恢复执行器空闲状态失败")
            self._released(pending)
        except Exception as error:
            self.journal.update(pending["task_id"], {"detail": str(error)})
            if pending.get("job"):
                self.node._reply(pending["job"], None, str(error))
            self.operation = None

    def _released(self, pending):
        self.journal.update(pending["task_id"], {"restored": True},
                            [{"seq": -3, "kind": "graph_restored", "detail": "原路线图已恢复，执行器空闲"}])
        if pending.get("job"):
            self.node._reply(pending["job"], {"task": self.get(pending["task_id"]),
                                              "accepted": False}, pending.get("error"))
        self.operation = None

    def _interrupt(self, document, patrol):
        pending = {"task_id": document["id"], "job": None, "stage": "interrupting",
                   "deadline": time.monotonic() + 5, "cancel_pending": 0,
                   "cancel_unknown": False, "stopped_since": None,
                   "release": "unlock" if patrol.get("state") == "previewing" else "none"}
        self.operation = pending
        # Cancel only action UUIDs recorded as belonging to this task.
        for goal in document.get("active_goals", []):
            action = goal["action"]
            if action not in {"compute_route", "follow_path", "navigate_to_pose", "spin", "wait"}:
                continue
            if action not in self.cancel_clients:
                self.cancel_clients[action] = self.node.create_client(CancelGoal, f"/{action}/_action/cancel_goal")
            client = self.cancel_clients[action]
            if client.service_is_ready():
                pending["cancel_pending"] += 1
                request = CancelGoal.Request()
                request.goal_info.goal_id.uuid = goal["uuid"]
                client.call_async(request).add_done_callback(
                    lambda future, owned=goal: self._on_orphan_cancel(future, pending, owned))
            else:
                pending["cancel_unknown"] = True
        self.journal.update(document["id"], {"status": "interrupted",
                                              "recovery_interrupted": True,
                                              "detail": "执行器心跳丢失、重启或任务状态不一致，停止且不重放任务"},
                            [{"seq": -1, "kind": "interrupted", "detail": "执行器状态失效，正在取消所属运动请求"}])

    def _on_orphan_cancel(self, future, pending, owned):
        if self.operation is not pending:
            return
        pending["cancel_pending"] -= 1
        try:
            response = future.result()
            if response.return_code == 1 or (response.return_code == 2 and owned.get("pending")):
                pending["cancel_unknown"] = True
        except Exception:
            pending["cancel_unknown"] = True

    def _poll_interrupt(self, pending):
        import math
        now = time.monotonic()
        age = ((self.node.get_clock().now() - Time.from_msg(self.odom.header.stamp)).nanoseconds / 1e9
               if self.odom else float("inf"))
        twist = self.odom.twist.twist if self.odom else None
        stopped = (not pending["cancel_pending"] and not pending["cancel_unknown"] and twist
                   and now - self.odom_wall <= 1 and 0 <= age <= 1
                   and math.hypot(twist.linear.x, twist.linear.y) <= .02 and abs(twist.angular.z) <= .02)
        if stopped:
            if pending["stopped_since"] is None:
                pending["stopped_since"] = now
            if now - pending["stopped_since"] >= .3:
                document = self.journal.get(pending["task_id"], details=False)
                patrol = self.node._status().get("patrol") or {}
                if not patrol.get("fresh"):
                    return
                if patrol.get("task_id") == document["id"]:
                    if patrol.get("state") != "finished":
                        if now - pending.get("last_stop_request", 0) >= 1:
                            self.clients["cancel"].call_async(Trigger.Request())
                            pending["last_stop_request"] = now
                        return
                    pending["release"] = "finish_web"
                elif patrol.get("state") not in {"idle", "previewing"}:
                    return
                else:
                    pending["release"] = "unlock" if patrol["state"] == "previewing" else "none"
                self.journal.update(document["id"], {"stop_confirmed": True, "stopped_at": utc_now()},
                                    [{"seq": -2, "kind": "stop_confirmed", "detail": "中断任务已确认停车"}])
                pending["locked"] = pending["release"] == "unlock"
                self._restore(document, pending["release"], pending)
                return
        else:
            pending["stopped_since"] = None
        if now > pending["deadline"]:
            self.journal.update(pending["task_id"], {"detail": "重启后的运动请求或停车未确认，保持任务锁定"})
            pending["deadline"] = now + 30

    def _control(self, job, snapshot):
        payload = job["payload"]
        task_id = str(uuid.UUID(payload["task_id"]))
        command_id = str(uuid.UUID(payload["command_id"]))
        action = job["kind"]
        if action not in {"pause", "resume", "cancel"}:
            raise ValueError("任务操作无效")
        try:
            document = self.journal.get(task_id)
        except KeyError:
            reservation = payload.get("task")
            if action != "cancel" or not isinstance(reservation, dict) or reservation.get("id") != task_id:
                raise
            document = self.journal.reserve_cancel(reservation)
        if document["status"] in TERMINAL:
            self.node._reply(job, {"task": document}, None)
            return
        result, created = self.journal.command(task_id, command_id, action)
        if not created:
            self.node._reply(job, result, result.get("error"))
            return
        patrol = snapshot.get("patrol") or {}
        try:
            if (action == "cancel" and self.operation
                    and self.operation["task_id"] == task_id
                    and self.operation["stage"] in {"locking", "parameters", "loading", "configuring", "starting"}):
                self.operation["cancel_requested"] = True
                self.journal.update(task_id, {"cancel_requested": True})
                result = {"task": document, "accepted": True, "pending": False}
                self.journal.finish_command(task_id, command_id, result)
                self.node._reply(job, result, None)
                return
            if self.operation or not patrol.get("fresh") or patrol.get("task_id") != task_id:
                raise ValueError("机器人任务状态未确认，不能操作其他任务")
            if action == "resume":
                loaded = snapshot.get("loaded_map") or {}
                if not snapshot.get("ready") or loaded.get("fingerprint") != document["fingerprint"]:
                    raise ValueError("定位或地图未就绪，不能继续任务")
            self.clients[action].call_async(Trigger.Request()).add_done_callback(
                lambda future: self._on_control(future, job, task_id, command_id))
        except Exception as error:
            result = {"error": str(error), "pending": False}
            self.journal.finish_command(task_id, command_id, result)
            raise

    def _on_control(self, future, job, task_id, command_id):
        try:
            response = future.result()
            if not response.success:
                raise ValueError(response.message)
            result = {"task": self.get(task_id), "pending": False, "accepted": True}
            self.journal.finish_command(task_id, command_id, result)
            self.node._reply(job, result, None)
        except Exception as error:
            result = {"error": str(error), "pending": False}
            self.journal.finish_command(task_id, command_id, result)
            self.node._reply(job, None, str(error))
