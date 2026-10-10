#!/usr/bin/env python3
"""Local ROS telemetry and motion-free Route Server previews for WebVizeSLAM."""

from __future__ import annotations

import json
import math
import queue
import tempfile
import threading
import time
import sqlite3
import os
import socket
import xml.etree.ElementTree as ET
from collections import deque
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FilePath

import rclpy
from nav2_msgs.action import ComputeRoute
from nav2_msgs.srv import SetRouteGraph
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import BatteryState, Imu, PointCloud2
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from super_slam.msg import LocalizationState
from tf2_ros import Buffer, TransformException, TransformListener
from task_manager import TaskManager
from navigation_settings import NavigationSettings
from web_mission import occupancy_fingerprint


def yaw_of(quaternion):
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y**2 + quaternion.z**2))


class CommandPendingError(ValueError):
    pass


def fingerprint(message):
    return occupancy_fingerprint(message)


class WebGateway(Node):
    def __init__(self):
        super().__init__("web_gateway")
        self.declare_parameter("http_port", 8765)
        self.declare_parameter("map_id", "")
        self.declare_parameter("version_id", "")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("freshness_sec", 1.0)
        self.declare_parameter("task_data_dir", "")
        for name, default in {"lidar_topic": "/lidar/points", "imu_topic": "/imu/data",
                              "odom_topic": "/wheel/odometry", "battery_topic": "/battery_state",
                              "robot_id": "", "robot_model": "", "robot_serial": ""}.items():
            self.declare_parameter(name, default)
        self._freshness = float(self.get_parameter("freshness_sec").value)
        if not 0.1 <= self._freshness <= 10:
            raise ValueError("freshness_sec must be between 0.1 and 10 seconds")
        self._tf = Buffer(node=self)
        self._listener = TransformListener(self._tf, self)
        self._lock = threading.Lock()
        self._snapshot = {"ready": False, "pose": None, "reason": "等待 ROS 数据"}
        self._snapshot_wall = 0.0
        self._localization = None
        self._localization_received = 0.0
        self._map = None
        self._paths = {}
        self._patrol = None
        self._patrol_received = 0.0
        self._commands = queue.Queue(maxsize=4)
        self._preview = None
        self._graph_dir = tempfile.TemporaryDirectory(prefix="web-slam-preview-")
        self._set_graph = self.create_client(SetRouteGraph, "/route_server/set_route_graph")
        self._compute = ActionClient(self, ComputeRoute, "/compute_route")
        self._graph_parameters = self.create_client(GetParameters, "/route_server/get_parameters")
        self._patrol_lock = self.create_client(SetBool, "/route_patrol/preview_lock")
        self._tasks = TaskManager(self, str(self.get_parameter("task_data_dir").value))
        self._navigation_settings = NavigationSettings(self)
        self._measurements = {}
        self._stopped_since = None
        self._sensor_times = {name: deque(maxlen=40) for name in ("lidar", "imu", "odometry", "battery")}
        self._robot_description_name = ""
        try:
            metadata = ET.parse(FilePath(get_package_share_directory("sim_robot_slam")) / "package.xml")
            self._software_version = f"sim_robot_slam {metadata.getroot().findtext('version')}"
        except (OSError, ET.ParseError, LookupError):
            self._software_version = None
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, retained)
        self.create_subscription(LocalizationState, "/lio/localization_state", self._on_localization, 10)
        self.create_subscription(String, "/route_patrol/status", self._on_patrol, retained)
        self.create_subscription(Path, "/route_patrol/path",
                                 lambda msg: self._on_path("mission", msg), retained)
        self.create_subscription(Path, "/route_patrol/active_path",
                                 lambda msg: self._on_path("active", msg), retained)
        self.create_subscription(Path, "/plan", lambda msg: self._on_path("plan", msg), 10)
        self.create_subscription(Path, "/lio/path", lambda msg: self._on_path("trail", msg), 10)
        for name, message_type, parameter in (
                ("lidar", PointCloud2, "lidar_topic"), ("imu", Imu, "imu_topic"),
                ("odometry", Odometry, "odom_topic"), ("battery", BatteryState, "battery_topic")):
            self.create_subscription(message_type, str(self.get_parameter(parameter).value),
                                     lambda message, key=name: self._on_measurement(key, message),
                                     qos_profile_sensor_data)
        self.create_subscription(String, "/robot_description", self._on_description, retained)
        self.create_timer(0.1, self._poll, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_map(self, message):
        if message.info.width * message.info.height != len(message.data) or not message.data:
            self._map = None
            return
        self._map = {"map_id": self.get_parameter("map_id").value,
                     "version_id": self.get_parameter("version_id").value,
                     "fingerprint": fingerprint(message), "frame_id": message.header.frame_id,
                     "width": message.info.width, "height": message.info.height,
                     "resolution": message.info.resolution}

    def _on_localization(self, message):
        self._localization = message
        self._localization_received = time.monotonic()

    def _on_description(self, message):
        if len(message.data) > 2 * 1024 * 1024:
            return
        try:
            root = ET.fromstring(message.data)
            if root.tag == "robot":
                self._robot_description_name = root.get("name", "")[:120]
        except ET.ParseError:
            self.get_logger().warning("机器人描述格式无效")

    def _on_measurement(self, name, message):
        now = time.monotonic()
        stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
        values, valid = {}, True
        if name == "lidar":
            valid = (message.width * message.height > 0 and message.point_step > 0
                     and message.row_step >= message.width * message.point_step
                     and len(message.data) >= message.row_step * message.height)
        elif name == "imu":
            valid = all(math.isfinite(value) for value in (
                message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z))
        elif name == "odometry":
            twist = message.twist.twist
            valid = all(math.isfinite(value) for value in (twist.linear.x, twist.linear.y, twist.angular.z))
            if valid:
                values = {"linear_x": twist.linear.x, "linear_y": twist.linear.y,
                          "linear_speed": math.hypot(twist.linear.x, twist.linear.y),
                          "angular_z": twist.angular.z}
            previous = self._measurements.get(name)
            if (not valid or values["linear_speed"] > .02 or abs(values["angular_z"]) > .02
                    or not previous or now - previous["wall"] > self._freshness):
                self._stopped_since = None
            if valid and values["linear_speed"] <= .02 and abs(values["angular_z"]) <= .02:
                if self._stopped_since is None:
                    self._stopped_since = now
            values["stopped"] = bool(self._stopped_since is not None and now - self._stopped_since >= .3)
        else:
            values = {"percentage": message.percentage * 100 if math.isfinite(message.percentage)
                      and 0 <= message.percentage <= 1 else None,
                      "voltage": message.voltage if math.isfinite(message.voltage) and message.voltage > 0 else None,
                      "present": bool(message.present)}
            valid = bool(message.present)
        self._measurements[name] = {"wall": now, "stamp": stamp, "valid": valid, "values": values}
        self._sensor_times[name].append(now)

    def _telemetry(self, now):
        sensors = {}
        ros_now = self.get_clock().now().nanoseconds / 1e9
        parameters = {"lidar": "lidar_topic", "imu": "imu_topic", "odometry": "odom_topic", "battery": "battery_topic"}
        for name, parameter in parameters.items():
            sample = self._measurements.get(name)
            wall_age = now - sample["wall"] if sample else None
            ros_age = ros_now - sample["stamp"] if sample else None
            freshness = max(10., self._freshness) if name == "battery" else self._freshness
            fresh = bool(sample and sample["valid"] and wall_age <= freshness
                         and 0 <= ros_age <= freshness)
            times = [stamp for stamp in self._sensor_times[name] if now - stamp <= 3]
            rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else None
            sensors[name] = {"fresh": fresh, "received": sample is not None,
                             "valid": bool(sample and sample["valid"]), "age_sec": wall_age,
                             "rate_hz": round(rate, 1) if fresh and rate is not None else None,
                             "topic": str(self.get_parameter(parameter).value)}
        velocity = {"fresh": sensors["odometry"]["fresh"]}
        battery = {"fresh": sensors["battery"]["fresh"]}
        if velocity["fresh"]:
            velocity.update(self._measurements["odometry"]["values"])
        if battery["fresh"]:
            battery.update(self._measurements["battery"]["values"])
        info = {"name": self._robot_description_name or None,
                "id": self.get_parameter("robot_id").value or None,
                "model": self.get_parameter("robot_model").value or self._robot_description_name or None,
                "serial": self.get_parameter("robot_serial").value or None,
                "software_version": self._software_version,
                "hostname": socket.gethostname(), "ros_distro": os.environ.get("ROS_DISTRO"),
                "base_frame": self.get_parameter("base_frame").value,
                "use_sim_time": bool(self.get_parameter("use_sim_time").value)}
        return {"sensors": sensors, "velocity": velocity, "battery": battery, "robot_info": info}

    def _on_patrol(self, message):
        try:
            document = json.loads(message.data)
            if not isinstance(document, dict) or not isinstance(document.get("state"), str):
                return
            self._patrol = document
            self._patrol_received = time.monotonic()
        except ValueError:
            self.get_logger().warning("巡检状态格式无效")

    def _path(self, message):
        frame = message.header.frame_id
        transform = None
        if frame != "map" and message.poses:
            transform = self._tf.lookup_transform("map", frame, Time()).transform
        stride = max(1, math.ceil(len(message.poses) / 1200))
        poses = message.poses[::stride]
        if message.poses and poses[-1] is not message.poses[-1]:
            poses.append(message.poses[-1])
        points = []
        for pose in poses:
            position = pose.pose.position
            x, y, z = position.x, position.y, position.z
            if transform:
                # Quaternion rotation, including roll/pitch of the map/world transform.
                q = transform.rotation
                tx = 2 * (q.y * z - q.z * y)
                ty = 2 * (q.z * x - q.x * z)
                tz = 2 * (q.x * y - q.y * x)
                x, y, z = (x + q.w * tx + q.y * tz - q.z * ty + transform.translation.x,
                           y + q.w * ty + q.z * tx - q.x * tz + transform.translation.y,
                           z + q.w * tz + q.x * ty - q.y * tx + transform.translation.z)
            if not all(math.isfinite(value) for value in (x, y, z)):
                raise ValueError("路径包含非有限坐标")
            points.append({"worldX": x, "worldY": y, "z": z})
        return {"frame_id": "map", "points": points}

    def _on_path(self, name, message):
        try:
            self._paths[name] = self._path(message)
        except (TransformException, ValueError):
            self._paths.pop(name, None)

    def snapshot(self):
        with self._lock:
            document = deepcopy(self._snapshot)
            if time.monotonic() - self._snapshot_wall > self._freshness:
                document.update(ready=False, pose=None, reason="ROS 状态更新已过期")
                if document.get("localization"):
                    document["localization"].update(fresh=False, valid=False)
                if document.get("patrol"):
                    document["patrol"]["fresh"] = False
                for sensor in document.get("sensors", {}).values():
                    sensor.update(fresh=False, rate_hz=None)
                document["velocity"] = {"fresh": False}
                document["battery"] = {"fresh": False}
                document["navigation_settings"] = {"available": False, "reason": "ROS 状态已过期"}
            return document

    def _status(self):
        now = time.monotonic()
        message = self._localization
        fresh = message is not None and now - self._localization_received <= self._freshness
        if fresh:
            stamp = Time.from_msg(message.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            fresh = 0 <= age <= self._freshness
        localization = {"fresh": fresh, "valid": bool(fresh and message.valid),
                        "state": message.state_name if message else "unknown",
                        "score": float(message.score) if message and math.isfinite(message.score) else None}
        pose, reason = None, "定位无效或数据过期"
        if localization["valid"]:
            try:
                transform = self._tf.lookup_transform(
                    "map", str(self.get_parameter("base_frame").value), Time())
                age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
                if not 0 <= age <= self._freshness:
                    raise ValueError("TF 数据过期")
                translation = transform.transform.translation
                yaw = yaw_of(transform.transform.rotation)
                if not all(math.isfinite(value) for value in (translation.x, translation.y, translation.z, yaw)):
                    raise ValueError("TF 坐标无效")
                pose = {"x": translation.x, "y": translation.y, "z": translation.z,
                        "yaw": yaw, "frame_id": "map"}
                reason = ""
            except (TransformException, ValueError) as error:
                reason = str(error)
        patrol = deepcopy(self._patrol)
        if patrol is not None:
            patrol["fresh"] = now - self._patrol_received <= self._freshness
        bound = bool(self._map and self._map["frame_id"] == "map"
                     and self._map["map_id"] and self._map["version_id"])
        if not bound:
            reason = "尚未绑定机器人实际加载的地图版本"
        preview_services = {
            "/route_server/set_route_graph": self._set_graph.service_is_ready(),
            "/compute_route (Action)": self._compute.server_is_ready(),
            "/route_server/get_parameters": self._graph_parameters.service_is_ready(),
            "/route_patrol/preview_lock": self._patrol_lock.service_is_ready(),
        }
        preview_missing = [name for name, ready in preview_services.items() if not ready]
        task_missing = self._tasks.unavailable_reasons()
        return {**self._telemetry(now), "ready": bool(pose and bound), "pose": pose, "localization": localization,
                "loaded_map": self._map, "paths": self._paths, "patrol": patrol,
                "preview_available": not preview_missing, "preview_missing": preview_missing,
                "preview_reason": "导航接口未就绪：" + "、".join(preview_missing) if preview_missing else "",
                "task_control_available": not task_missing, "task_missing": task_missing,
                "task_reason": "任务接口未就绪：" + "、".join(task_missing) if task_missing else "",
                "navigation_settings": self._navigation_settings.snapshot(),
                "task_operation_pending": self._tasks.operation is not None or self._navigation_settings.applying,
                "reason": reason}

    def _poll(self):
        self._navigation_settings.poll()
        snapshot = self._status()
        try:
            self._tasks.poll(snapshot)
        except (sqlite3.Error, OSError, ValueError, TypeError) as error:
            now = time.monotonic()
            if now - getattr(self, "_last_store_error", 0) < 1:
                return
            self._last_store_error = now
            self.get_logger().error(f"任务记录更新失败：{error}")
            patrol = snapshot.get("patrol") or {}
            if (patrol.get("fresh") and patrol.get("task_id")
                    and patrol.get("state") not in {"idle", "finished"}
                    and self._tasks.clients["cancel"].service_is_ready()):
                self._tasks.clients["cancel"].call_async(Trigger.Request())
        if snapshot.get("patrol"):
            snapshot["patrol"].pop("events", None)
        with self._lock:
            self._snapshot = deepcopy(snapshot)
            self._snapshot_wall = time.monotonic()
        if self._preview and time.monotonic() > self._preview["deadline"]:
            if self._preview["stage"] in {"restoring", "unlocking"}:
                pending, self._preview = self._preview, None
                self._reply(pending["job"], None, "恢复原路线图或释放预览锁超时，请重启导航与巡检节点")
            else:
                self._finish_preview(None, "路线规划超时")
        try:
            job = self._commands.get_nowait()
        except queue.Empty:
            return
        if job["abandoned"].is_set():
            return
        if job.get("kind") == "navigation_settings":
            try:
                self._navigation_settings.start(job, snapshot)
            except (ValueError, TypeError) as error:
                self._reply(job, None, str(error))
            return
        if self._navigation_settings.applying:
            self._reply(job, None, "导航参数正在下发或恢复中")
            return
        if job.get("kind", "preview") != "preview":
            self._tasks.process(job, snapshot)
            return
        if self._tasks.operation:
            self._reply(job, None, "任务下发或路线恢复正在处理中")
            return
        if self._preview:
            self._reply(job, None, "已有路线预览正在规划")
            return
        try:
            payload = job["payload"]
            route = payload["route"]
            if not snapshot["ready"] or not snapshot["preview_available"]:
                raise ValueError(snapshot["reason"] or snapshot.get("preview_reason") or "Route Server 未就绪")
            loaded = snapshot["loaded_map"]
            if (route["map_id"], route["version_id"], payload["fingerprint"]) != (
                    loaded["map_id"], loaded["version_id"], loaded["fingerprint"]):
                raise ValueError("网页地图与机器人实际加载的栅格或版本不一致")
            patrol = snapshot["patrol"]
            if not patrol or not patrol["fresh"] or patrol["state"] != "idle":
                raise ValueError("巡检节点未就绪或正在执行任务，不能更换预览路线图")
            graph = payload["graph"]
            if not isinstance(graph, dict) or not 2 <= len(graph.get("features", [])) <= 12000:
                raise ValueError("路线图格式无效")
            stations = payload["station_ids"]
            if not isinstance(stations, list) or not 2 <= len(stations) <= 24:
                raise ValueError("巡检点列表无效")
            filename = FilePath(self._graph_dir.name) / "preview.geojson"
            filename.write_text(json.dumps(graph, allow_nan=False), encoding="utf-8")
            self._preview = {"job": job, "deadline": time.monotonic() + 16,
                             "start": stations[0], "goal": stations[-1], "handle": None,
                             "stage": "locking", "locked": False,
                             "original": None, "filename": str(filename)}
            request = SetBool.Request()
            request.data = True
            self._patrol_lock.call_async(request).add_done_callback(
                lambda future, pending=self._preview: self._on_lock(future, pending))
        except (KeyError, TypeError, ValueError, OSError) as error:
            self._reply(job, None, str(error))
            self._preview = None

    def _on_lock(self, future, pending):
        try:
            response = future.result()
            if self._preview is not pending:
                # A late lock response must not leave the executor reserved forever.
                if response.success:
                    request = SetBool.Request()
                    request.data = False
                    self._patrol_lock.call_async(request)
                return
            if not response.success:
                raise ValueError(response.message)
            pending["locked"] = True
            pending["stage"] = "parameters"
            request = GetParameters.Request()
            request.names = ["graph_filepath"]
            self._graph_parameters.call_async(request).add_done_callback(
                lambda result: self._on_parameters(result, pending))
        except Exception as error:
            self._finish_preview(None, str(error))

    def _on_parameters(self, future, pending):
        if self._preview is not pending or pending["stage"] != "parameters":
            return
        try:
            values = future.result().values
            original = values[0].string_value if values else ""
            if not original:
                raise ValueError("导航未配置原路线图，无法保证预览后恢复，请先通过 route_graph 加载路线")
            pending["original"] = original
            pending["stage"] = "setting"
            request = SetRouteGraph.Request()
            request.graph_filepath = pending["filename"]
            self._set_graph.call_async(request).add_done_callback(
                lambda result: self._on_graph(result, pending))
        except Exception as error:
            self._finish_preview(None, str(error))

    def _on_graph(self, future, pending):
        if self._preview is not pending or pending["stage"] != "setting":
            return
        try:
            if not future.result().success:
                raise ValueError("Route Server 拒绝加载路线图")
            goal = ComputeRoute.Goal()
            goal.start_id = self._preview["start"]
            goal.goal_id = self._preview["goal"]
            goal.use_poses = False
            goal.use_start = False
            pending["stage"] = "computing"
            self._compute.send_goal_async(goal).add_done_callback(
                lambda result: self._on_goal(result, pending))
        except Exception as error:
            self._finish_preview(None, str(error))

    def _on_goal(self, future, pending):
        try:
            handle = future.result()
            if self._preview is not pending or pending["stage"] != "computing":
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                raise ValueError("Route Server 拒绝规划请求")
            self._preview["handle"] = handle
            handle.get_result_async().add_done_callback(lambda result: self._on_result(result, pending))
        except Exception as error:
            self._finish_preview(None, str(error))

    def _on_result(self, future, pending):
        if self._preview is not pending or pending["stage"] != "computing":
            return
        try:
            wrapped = future.result()
            if wrapped.status != 4 or wrapped.result.error_code != 0 or not wrapped.result.path.poses:
                raise ValueError(f"路线规划失败，错误码 {wrapped.result.error_code}")
            path = self._path(wrapped.result.path)
            self._finish_preview({"planned": True, "path": path}, None)
        except Exception as error:
            self._finish_preview(None, str(error))

    @staticmethod
    def _reply(job, result, error):
        job["result"], job["error"] = result, error
        job["done"].set()

    def _finish_preview(self, result, error):
        if not self._preview:
            return
        pending = self._preview
        if pending["stage"] in {"restoring", "unlocking"}:
            return
        if error and pending["handle"]:
            pending["handle"].cancel_goal_async()
        pending["result"], pending["error"] = result, error
        pending["deadline"] = time.monotonic() + 4
        if pending["original"]:
            pending["stage"] = "restoring"
            request = SetRouteGraph.Request()
            request.graph_filepath = pending["original"]
            self._set_graph.call_async(request).add_done_callback(
                lambda future: self._on_restore(future, pending))
        else:
            self._unlock(pending)

    def _on_restore(self, future, pending):
        if self._preview is not pending:
            return
        try:
            if not future.result().success:
                raise ValueError("原路线图恢复失败；保持巡检锁定，请重启导航与巡检节点")
        except Exception as error:
            self._preview = None
            self._reply(pending["job"], None, str(error))
            return
        self._unlock(pending)

    def _unlock(self, pending):
        if not pending["locked"]:
            self._preview = None
            self._reply(pending["job"], pending["result"], pending["error"])
            return
        pending["stage"] = "unlocking"
        request = SetBool.Request()
        request.data = False
        self._patrol_lock.call_async(request).add_done_callback(
            lambda future: self._on_unlock(future, pending))

    def _on_unlock(self, future, pending):
        if self._preview is not pending:
            return
        try:
            if not future.result().success:
                raise ValueError("释放预览锁失败")
            self._reply(pending["job"], pending["result"], pending["error"])
        except Exception as error:
            self._reply(pending["job"], None, str(error))
        self._preview = None

    def command(self, kind, payload):
        job = {"kind": kind, "payload": payload, "done": threading.Event(), "abandoned": threading.Event()}
        self._commands.put_nowait(job)
        if not job["done"].wait(20):
            job["abandoned"].set()
            raise CommandPendingError("请求超时，需查询任务状态确认结果")
        if job["error"]:
            raise ValueError(job["error"])
        return job["result"]

    def preview(self, payload):
        return self.command("preview", payload)


def handler_for(node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *args):
            pass

        def respond(self, status, document):
            data = json.dumps(document, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/navigation-settings":
                self.respond(200, node.snapshot().get("navigation_settings", {"available": False}))
                return
            if self.path == "/tasks":
                self.respond(200, {"tasks": node._tasks.list()})
                return
            if self.path.startswith("/tasks/"):
                try:
                    self.respond(200, {"task": node._tasks.get(self.path.removeprefix("/tasks/"))})
                except KeyError:
                    self.respond(404, {"detail": "任务不存在"})
                return
            if self.path != "/status":
                self.respond(404, {"detail": "接口不存在"})
                return
            self.respond(200, node.snapshot())

        def do_POST(self):
            if self.headers.get("Origin"):
                self.respond(403, {"detail": "网关仅接受本机服务调用"})
                return
            if self.path not in {"/preview", "/tasks", "/pause", "/resume", "/cancel", "/delete", "/navigation-settings"}:
                self.respond(404, {"detail": "接口不存在"})
                return
            try:
                self.connection.settimeout(5)
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 2 * 1024 * 1024:
                    raise ValueError("请求大小无效")
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError("请求必须是对象")
                kind = ("navigation_settings" if self.path == "/navigation-settings" else
                        "preview" if self.path == "/preview" else "start" if self.path == "/tasks" else self.path[1:])
                self.respond(200, node.command(kind, payload))
            except CommandPendingError as error:
                self.respond(409 if self.path == "/preview" else 202,
                             {"pending": True, "detail": str(error)})
            except (ValueError, queue.Full) as error:
                self.respond(409, {"detail": str(error) or "网关繁忙"})
            except (OSError, UnicodeError):
                return

    return Handler


def main():
    rclpy.init()
    node = WebGateway()
    server = ThreadingHTTPServer(("127.0.0.1", int(node.get_parameter("http_port").value)), handler_for(node))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f"Web 网关仅监听 http://127.0.0.1:{server.server_port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node._graph_dir.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
