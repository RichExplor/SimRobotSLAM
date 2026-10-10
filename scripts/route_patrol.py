#!/usr/bin/env python3

"""Execute a route graph path with Nav2's Route Server and controller."""

import math
import json
import time
import uuid
import sqlite3
from datetime import datetime, timezone
from collections.abc import Sequence
from copy import deepcopy
from dataclasses import dataclass
from dataclasses import replace
from pathlib import Path as FilePath

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputeRoute, FollowPath, NavigateToPose, Spin, Wait
from nav2_msgs.msg import RouteNode
from nav_msgs.msg import Odometry, Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from super_slam.msg import LocalizationState
from unique_identifier_msgs.msg import UUID
from task_journal import TaskJournal
from web_mission import occupancy_fingerprint, parse_mission, validate_footprints, yaw_of


@dataclass(frozen=True)
class PointTask:
    """One validated inspection action at a graph node."""

    kind: str
    timeout_sec: float
    angle_rad: float = 0.0
    duration_sec: float = 0.0


def load_point_tasks(filename: str) -> dict[int, list[PointTask]]:
    """Load station tasks; an empty filename disables inspection actions."""
    if not filename:
        return {}
    try:
        document = yaml.safe_load(FilePath(filename).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"任务 YAML 格式错误：{exc}") from exc
    if not isinstance(document, dict):
        raise ValueError("任务配置必须包含 points 映射。")
    if set(document) - {"failure_policy", "points"}:
        raise ValueError("任务配置仅支持 failure_policy 和 points。")
    if document.get("failure_policy", "stop") != "stop":
        raise ValueError("目前 failure_policy 仅支持 stop。")
    points = document.get("points")
    if not isinstance(points, dict):
        raise ValueError("points 必须是节点 ID 到任务列表的映射。")

    def number(value, label, *, positive=False):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{label} 必须是有限数值。")
        try:
            result = float(value)
        except OverflowError as exc:
            raise ValueError(f"{label} 超出数值范围。") from exc
        if not math.isfinite(result) or result < 0 or (positive and result == 0):
            raise ValueError(f"{label} 必须是{'正数' if positive else '非负数'}。")
        if result >= 2**31 - 1:
            raise ValueError(f"{label} 超出 ROS Duration 的范围。")
        return result

    tasks = {}
    for raw_id, items in points.items():
        if not str(raw_id).isdigit() or not 0 <= int(raw_id) <= 65535:
            raise ValueError(f"无效的巡检节点 ID：{raw_id}。")
        node_id = int(raw_id)
        if node_id in tasks:
            raise ValueError(f"巡检节点 {node_id} 重复。")
        if not isinstance(items, list):
            raise ValueError(f"节点 {node_id} 的任务必须是列表。")
        tasks[node_id] = []
        for item in items:
            if not isinstance(item, dict):
                raise ValueError(f"节点 {node_id} 的每项任务必须是映射。")
            kind = item.get("type")
            if kind == "spin":
                if set(item) - {"type", "angle_deg", "timeout_sec"}:
                    raise ValueError(f"节点 {node_id} 的 spin 包含未知配置项。")
                angle = item.get("angle_deg")
                if isinstance(angle, bool) or not isinstance(angle, (int, float)):
                    raise ValueError(f"节点 {node_id} 的 angle_deg 必须是有限数值。")
                try:
                    angle_rad = math.radians(angle)
                except OverflowError as exc:
                    raise ValueError(f"节点 {node_id} 的 angle_deg 超出数值范围。") from exc
                if not math.isfinite(angle_rad) or abs(angle_rad) > 3.402823e38:
                    raise ValueError(f"节点 {node_id} 的 angle_deg 超出 Spin 的有效范围。")
                timeout = number(item.get("timeout_sec", 30.0), "timeout_sec", positive=True)
                task = PointTask(kind, timeout, angle_rad=angle_rad)
            elif kind == "wait":
                if set(item) - {"type", "duration_sec", "timeout_sec"}:
                    raise ValueError(f"节点 {node_id} 的 wait 包含未知配置项。")
                duration = number(item.get("duration_sec"), "duration_sec")
                timeout = number(
                    item.get("timeout_sec", duration + 10.0), "timeout_sec", positive=True
                )
                if timeout <= duration:
                    raise ValueError("wait 的 timeout_sec 必须大于 duration_sec。")
                task = PointTask(kind, timeout, duration_sec=duration)
            else:
                raise ValueError(f"节点 {node_id} 包含不支持的任务类型：{kind}。")
            tasks[node_id].append(task)
    return tasks


def split_route_path(path: Path, nodes: Sequence[RouteNode]) -> list[Path]:
    """Split a dense route at its ordered graph nodes, preserving edge geometry."""
    if len(nodes) < 2 or len(path.poses) < 2:
        raise ValueError("路线至少需要两个图节点和两个路径点。")

    def distance_squared(pose, node):
        point = pose.pose.position
        return (point.x - node.position.x) ** 2 + (point.y - node.position.y) ** 2

    # Corner smoothing is disabled: graph nodes must lie on the returned path.
    if (
        distance_squared(path.poses[0], nodes[0]) > 0.1**2
        or distance_squared(path.poses[-1], nodes[-1]) > 0.1**2
    ):
        raise ValueError("路线起终点与图节点不一致，无法安全分段。")

    boundaries = [0]
    for node in nodes[1:-1]:
        candidates = range(boundaries[-1] + 1, len(path.poses) - 1)
        if not candidates:
            raise ValueError("图节点的顺序与路径点不一致。")
        index = min(candidates, key=lambda i: distance_squared(path.poses[i], node))
        if distance_squared(path.poses[index], node) > 0.1**2:
            raise ValueError(f"路径未经过节点 {node.nodeid}；请关闭路线拐角平滑。")
        boundaries.append(index)
    boundaries.append(len(path.poses) - 1)

    segments = []
    for start, end in zip(boundaries, boundaries[1:]):
        segment = Path()
        segment.header = deepcopy(path.header)
        for pose in path.poses[start : end + 1]:
            point = pose.pose.position
            if segment.poses:
                previous = segment.poses[-1].pose.position
                if math.hypot(point.x - previous.x, point.y - previous.y) < 1e-6:
                    continue
            segment.poses.append(deepcopy(pose))
        if len(segment.poses) < 2:
            raise ValueError("路线包含长度为零的边。")

        # Each edge ends facing its own tangent. The next FollowPath goal then
        # triggers RotationShim again before the robot enters the next edge.
        for i, pose in enumerate(segment.poses):
            first = segment.poses[min(i, len(segment.poses) - 2)].pose.position
            second = segment.poses[min(i + 1, len(segment.poses) - 1)].pose.position
            yaw = math.atan2(second.y - first.y, second.x - first.x)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            pose.header = deepcopy(segment.header)
        segments.append(segment)
    return segments


class RoutePatrol(Node):
    """Start and cancel one graph-based route traversal on request."""

    def __init__(self, **kwargs):
        super().__init__("route_patrol", **kwargs)
        self.declare_parameter("start_id", 1)
        self.declare_parameter("goal_id", 10)
        self.declare_parameter("via_ids", "")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")
        self.declare_parameter("task_config", "")
        self.declare_parameter("odom_topic", "/wheel/odometry")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("start_tolerance", 0.25)
        self.declare_parameter("stop_timeout_sec", 5.0)
        self.declare_parameter("stop_settle_time_sec", 0.3)
        self.declare_parameter("web_mission", "")
        self.declare_parameter("web_freshness_sec", 1.0)
        self.declare_parameter("costmap_freshness_sec", 3.0)
        self.declare_parameter("resume_tolerance", 0.5)

        self._start_id = int(self.get_parameter("start_id").value)
        self._goal_id = int(self.get_parameter("goal_id").value)
        self._controller_id = str(self.get_parameter("controller_id").value)
        self._goal_checker_id = str(
            self.get_parameter("goal_checker_id").value
        )
        self._progress_checker_id = str(
            self.get_parameter("progress_checker_id").value
        )
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._start_tolerance = float(self.get_parameter("start_tolerance").value)
        self._stop_timeout = float(self.get_parameter("stop_timeout_sec").value)
        self._stop_settle_time = float(self.get_parameter("stop_settle_time_sec").value)
        if not all(
            math.isfinite(value) and value > 0
            for value in (self._start_tolerance, self._stop_timeout, self._stop_settle_time)
        ) or self._stop_settle_time >= self._stop_timeout:
            raise ValueError("起点容差、停车超时与稳定时间必须为正数，稳定时间须小于超时。")

        self._compute_client = ActionClient(self, ComputeRoute, "/compute_route")
        self._follow_client = ActionClient(self, FollowPath, "/follow_path")
        self._navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._spin_client = ActionClient(self, Spin, "/spin")
        self._wait_client = ActionClient(self, Wait, "/wait")
        self._path_publisher = self.create_publisher(
            Path,
            "/route_patrol/path",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._active_path_publisher = self.create_publisher(
            Path, "/route_patrol/active_path",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._status_publisher = self.create_publisher(
            String, "/route_patrol/status",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._run_id = None
        self._last_outcome = None
        self._last_summary = {}
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._odom_speed = None
        self._odom_received_at = None
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self._state = "idle"
        self._executor_instance = str(uuid.uuid4())
        self._web_task = None
        self._pending_web_task = None
        self._last_web_task_id = None
        self._events = []
        self._event_seq = 0
        self._journal = None
        self._journal_failed = False
        self._pending_goal = None
        self._completed_points = 0
        self._send_pending = False
        self._halt_kind = None
        self._halt_detail = ""
        self._resume_phase = None
        self._halt_deadline = None
        self._halt_action_elapsed = None
        self._odom_wall = None
        self._odom_stamp = None
        self._map_fingerprint = None
        self._localization = None
        self._localization_wall = 0.0
        self._costmap = None
        self._costmap_wall = 0.0
        self._footprint = []
        self._footprint_wall = 0.0
        self._footprint_stamp = None
        self._aligned = False
        self._action_started = None
        self._spin_progress = 0.0
        self._spin_last_yaw = None
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LocalizationState, "/lio/localization_state", self._on_localization, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_costmap, retained)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, retained)
        self.create_subscription(PolygonStamped, "/global_costmap/published_footprint", self._on_footprint, 10)
        self.add_on_set_parameters_callback(self._configure_web_task)
        self._cancel_requested = False
        self._compute_goal_handle = None
        self._follow_goal_handle = None
        self._last_feedback_log_time = 0.0
        self._segments = []
        self._route_node_ids = []
        self._segment_index = 0
        self._planning_ids = []
        self._planning_index = 0
        self._planned_path_points = 0
        self._point_tasks = {}
        self._station_node_id = None
        self._station_tasks = []
        self._task_index = 0
        self._after_station = None
        self._active_task = None
        self._mission_goal_handle = None
        self._mission_deadline = None
        self._mission_label = ""
        self._abort_reason = None
        self._stop_deadline = None
        self._stopped_since = None
        self._last_task_feedback_log_time = 0.0
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._flow_timer = self.create_timer(0.1, self._poll_flow, clock=steady_clock)
        self._status_timer = self.create_timer(0.2, self._publish_status, clock=steady_clock)

        self.create_service(Trigger, "/route_patrol/start", self._start_route)
        self.create_service(Trigger, "/route_patrol/cancel", self._cancel_route)
        self.create_service(SetBool, "/route_patrol/preview_lock", self._preview_lock)
        self.create_service(Trigger, "/route_patrol/start_web", self._start_web)
        self.create_service(Trigger, "/route_patrol/pause", self._pause_web)
        self.create_service(Trigger, "/route_patrol/resume", self._resume_web)
        self.create_service(Trigger, "/route_patrol/finish_web", self._finish_web)
        self.get_logger().info(
            f"Ready. Waiting for /route_patrol/start; route "
            f"{self._start_id} -> {self._goal_id}."
        )

    def _start_route(self, _request, response):
        if self._state != "idle":
            response.success = False
            response.message = f"路线任务正在运行（状态：{self._state}）。"
            return response

        if not 0 <= self._start_id <= 65535 or not 0 <= self._goal_id <= 65535:
            response.success = False
            response.message = "start_id 和 goal_id 必须在 0 到 65535 之间。"
            return response

        try:
            via_text = str(self.get_parameter("via_ids").value).strip()
            via_ids = [int(value.strip()) for value in via_text.split(",")] if via_text else []
            if any(not 0 <= node_id <= 65535 for node_id in via_ids):
                raise ValueError("中间节点 ID 必须在 0 到 65535 之间。")
        except ValueError as exc:
            response.success = False
            response.message = f"via_ids 必须是逗号分隔的节点 ID：{exc}"
            return response

        planning_ids = []
        for node_id in [self._start_id, *via_ids, self._goal_id]:
            if not planning_ids or node_id != planning_ids[-1]:
                planning_ids.append(node_id)
        if len(planning_ids) < 2:
            response.success = False
            response.message = "起终点相同时必须指定不同的中间节点，例如 via_ids:=9。"
            return response

        if (
            self.get_parameter("use_sim_time").value
            and self.get_clock().now().nanoseconds == 0
        ):
            response.success = False
            response.message = "仿真时钟尚未就绪；请等待首帧 /clock 后再开始巡检。"
            return response

        try:
            self._point_tasks = load_point_tasks(str(self.get_parameter("task_config").value))
        except (OSError, ValueError) as exc:
            response.success = False
            response.message = f"无法加载巡检任务配置：{exc}"
            self.get_logger().error(response.message)
            return response

        if not self._compute_client.server_is_ready():
            response.success = False
            response.message = (
                "/compute_route 尚未就绪；请等待 Nav2 定位完成并激活。"
            )
            return response

        if not self._follow_client.server_is_ready():
            response.success = False
            response.message = "/follow_path 尚未就绪；请等待 Nav2 完全激活。"
            return response

        self._state = "computing"
        self._run_id = str(uuid.uuid4())
        self._last_web_task_id = None
        self._events = []
        self._last_outcome = None
        self._last_summary = {}
        self._cancel_requested = False
        self._compute_goal_handle = None
        self._follow_goal_handle = None
        self._last_feedback_log_time = 0.0
        self._planning_ids = planning_ids
        self._planning_index = 0
        self._planned_path_points = 0
        self._segments = []
        self._route_node_ids = []
        self._publish_patrol_path()

        if not self._submit_compute_leg():
            response.success = False
            response.message = "无法提交路线规划请求，请查看巡检节点日志。"
            return response
        response.success = True
        response.message = (
            "已提交巡检路线：" + " -> ".join(str(node_id) for node_id in planning_ids) + "。"
        )
        self.get_logger().info(response.message)
        return response

    def _preview_lock(self, request, response):
        if request.data:
            if self._state != "idle":
                response.success = False
                response.message = "巡检执行器忙，不能预览新路线。"
                return response
            self._state = "previewing"
        elif self._state == "previewing":
            self._state = "idle"
        else:
            response.success = False
            response.message = "没有持有路线预览锁。"
            return response
        response.success = True
        return response

    def _configure_web_task(self, parameters):
        try:
            candidate = None
            for parameter in parameters:
                if parameter.name == "web_mission":
                    if self._state != "previewing":
                        raise ValueError("必须先锁定空闲巡检执行器")
                    candidate = parse_mission(parameter.value)
                elif parameter.name in {"web_freshness_sec", "costmap_freshness_sec", "resume_tolerance"}:
                    value = parameter.value
                    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not .1 <= value <= 10:
                        raise ValueError(f"{parameter.name} 必须在 0.1～10 之间")
            if candidate is not None:
                self._pending_web_task = candidate
            return SetParametersResult(successful=True)
        except (ValueError, TypeError, KeyError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    def _event(self, kind, detail):
        if self._web_task:
            self._event_seq += 1
            event = {"seq": self._event_seq, "kind": kind, "detail": detail,
                     "ros_time": self._now_seconds(), "time": datetime.now(timezone.utc).isoformat()}
            self._events.append(event)
            self._events = self._events[-2048:]
            if self._journal and not self._journal_failed:
                status = (self._last_outcome if self._last_outcome else
                          "paused" if self._state == "paused" else "pausing" if self._state == "pausing" else
                          "cancelling" if self._halt_kind else "running")
                try:
                    self._journal.update(self._web_task["task_id"], {
                        "status": status, "state": self._state, "event_seq": self._event_seq,
                        "completed_points": self._completed_points, "active_goals": self._owned_goals(),
                        "executor_instance": self._executor_instance, "detail": self._halt_detail}, [event])
                except (OSError, sqlite3.Error) as error:
                    self._journal_failed = True
                    self.get_logger().error(f"任务记录无法写入：{error}")
                    self._request_halt("failed", "任务记录无法写入，停止执行")
            self._publish_status()

    def _owned_goals(self):
        goals = [self._pending_goal] if self._pending_goal else []
        for name, handle in (("compute_route", self._compute_goal_handle),
                             ("follow_path", self._follow_goal_handle),
                             (getattr(self, "_mission_action_name", ""), self._mission_goal_handle)):
            if handle is not None:
                goals.append({"action": name, "uuid": [int(value) for value in handle.goal_id.uuid], "pending": False})
        return goals

    def _send_goal(self, client, goal, name, feedback_callback=None):
        identifier = UUID(uuid=list(uuid.uuid4().bytes))
        self._pending_goal = {"action": name, "uuid": [int(value) for value in identifier.uuid], "pending": True}
        self._event("goal_submitted", f"提交 {name} 请求")
        if self._web_task and self._journal_failed:
            self._pending_goal = None
            raise RuntimeError("任务记录写入失败，未提交运动请求")
        try:
            return client.send_goal_async(goal, feedback_callback=feedback_callback, goal_uuid=identifier)
        except Exception:
            self._pending_goal = None
            raise

    def _on_map(self, message):
        self._map_fingerprint = occupancy_fingerprint(message)

    def _on_localization(self, message):
        self._localization = message
        self._localization_wall = time.monotonic()

    def _on_costmap(self, message):
        self._costmap = message
        self._costmap_wall = time.monotonic()

    def _on_footprint(self, message):
        try:
            if message.header.frame_id == self._base_frame:
                points = [(point.x, point.y) for point in message.polygon.points]
            else:
                transform = self._tf_buffer.lookup_transform(
                    message.header.frame_id, self._base_frame, Time.from_msg(message.header.stamp)).transform
                yaw = yaw_of(transform.rotation)
                c, s = math.cos(yaw), math.sin(yaw)
                points = [(c * (point.x - transform.translation.x) + s * (point.y - transform.translation.y),
                           -s * (point.x - transform.translation.x) + c * (point.y - transform.translation.y))
                          for point in message.polygon.points]
            if len(points) < 3 or any(not math.isfinite(v) for point in points for v in point):
                return
            self._footprint = points
            self._footprint_wall = time.monotonic()
            self._footprint_stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
        except TransformException:
            return

    def _web_ready(self):
        freshness = float(self.get_parameter("web_freshness_sec").value)
        mission = self._web_task or self._pending_web_task
        if mission and self._map_fingerprint != mission["fingerprint"]:
            raise ValueError("机器人地图发生变化或尚未收到地图")
        message = self._localization
        if (message is None or not message.valid or time.monotonic() - self._localization_wall > freshness
                or not 0 <= self._now_seconds() - Time.from_msg(message.header.stamp).nanoseconds / 1e9 <= freshness):
            raise ValueError("定位无效或量测数据过期")
        transform = self._tf_buffer.lookup_transform("map", self._base_frame, Time())
        if not all(math.isfinite(value) for value in (
                transform.transform.translation.x, transform.transform.translation.y,
                yaw_of(transform.transform.rotation))):
            raise ValueError("机器人 TF 坐标无效")
        if not 0 <= self._now_seconds() - Time.from_msg(transform.header.stamp).nanoseconds / 1e9 <= freshness:
            raise ValueError("机器人 TF 已过期")
        cost_age = float(self.get_parameter("costmap_freshness_sec").value)
        if (self._costmap is None or time.monotonic() - self._costmap_wall > cost_age
                or not 0 <= self._now_seconds() - Time.from_msg(self._costmap.header.stamp).nanoseconds / 1e9 <= cost_age):
            raise ValueError("全局代价地图已过期")
        if (not self._footprint or time.monotonic() - self._footprint_wall > cost_age
                or not 0 <= self._now_seconds() - self._footprint_stamp <= cost_age):
            raise ValueError("机器人轮廓已过期")
        return transform.transform

    def _start_web(self, _request, response):
        if self._state != "previewing" or self._pending_web_task is None:
            response.message = "Web 任务尚未配置或执行器未锁定"
            return response
        try:
            self._web_ready()
            directory = FilePath(self._pending_web_task["journal_dir"])
            if not directory.is_absolute() or not (directory / "tasks.sqlite3").is_file():
                raise ValueError("机器人任务记录目录未准备好")
            journal = TaskJournal(directory)
            recorded = journal.get(self._pending_web_task["task_id"], details=False)
            if recorded["status"] != "dispatching" or recorded.get("restored"):
                raise ValueError("任务已经接受、结束或中断，不能重复执行")
            if recorded["fingerprint"] != self._pending_web_task["fingerprint"]:
                raise ValueError("持久化任务快照与地图不一致")
            if not all(client.server_is_ready() for client in (
                    self._compute_client, self._follow_client, self._navigate_client,
                    self._spin_client, self._wait_client)):
                raise ValueError("导航、跟随或点位动作服务未就绪")
        except (ValueError, TransformException, KeyError, OSError, sqlite3.Error) as error:
            response.message = str(error)
            return response
        self._web_task = self._pending_web_task
        self._journal = journal
        self._journal_failed = False
        self._pending_web_task = None
        self._last_web_task_id = self._web_task["task_id"]
        self._run_id = self._last_web_task_id
        self._events = []
        self._event_seq = 0
        self._completed_points = 0
        self._last_outcome = None
        self._last_summary = {}
        self._cancel_requested = False
        self._halt_kind = None
        self._halt_detail = ""
        self._resume_phase = None
        self._planning_ids = [station["node_id"] for station in self._web_task["stations"]]
        self._point_tasks = {}
        for station in self._web_task["stations"]:
            self._point_tasks[station["node_id"]] = [
                PointTask("wait", action["duration_sec"] + 10, duration_sec=action["duration_sec"])
                if action["type"] == "wait" else
                PointTask("spin", action["timeout_sec"], angle_rad=math.radians(action["angle_deg"]))
                for action in station["actions"]]
        self._planning_index = 0
        self._segments = []
        self._route_node_ids = []
        self._segment_index = 0
        self._planned_path_points = 0
        self._state = "computing"
        self._publish_patrol_path()
        self._event("accepted", "机器人已接受任务，开始规划")
        self._submit_compute_leg()
        response.success = True
        response.message = self._run_id
        return response

    def _finish_web(self, _request, response):
        if self._state != "finished":
            response.message = "任务尚未完成停车确认"
            return response
        self._web_task = None
        self._journal = None
        self._pending_web_task = None
        self._state = "idle"
        self._publish_status()
        response.success = True
        return response

    def _submit_compute_leg(self):
        if self._cancel_requested:
            self._finish_cancelled()
            return False

        goal = ComputeRoute.Goal()
        goal.start_id = self._planning_ids[self._planning_index]
        goal.goal_id = self._planning_ids[self._planning_index + 1]
        goal.use_start = False
        goal.use_poses = False

        self.get_logger().info(
            f"规划第 {self._planning_index + 1}/{len(self._planning_ids) - 1} 段路线："
            f"节点 {goal.start_id} -> {goal.goal_id}。"
        )
        try:
            self._send_pending = True
            future = self._send_goal(self._compute_client, goal, "compute_route")
        except Exception as exc:
            self._send_pending = False
            self._finish_with_error(f"提交路线规划请求失败：{exc}")
            return False
        future.add_done_callback(self._on_compute_goal_response)
        return True

    def _on_compute_goal_response(self, future):
        self._send_pending = False
        self._pending_goal = None
        try:
            goal_handle = future.result()
        except Exception as exc:  # ROS futures may report transport errors here.
            self._finish_with_error(f"路线规划请求异常：{exc}")
            return

        if not goal_handle.accepted:
            self._finish_with_error("Route Server 拒绝了路线规划请求。")
            return

        self._compute_goal_handle = goal_handle
        self._event("goal_accepted", "路线规划请求已接受")
        if self._cancel_requested or self._halt_kind:
            goal_handle.cancel_goal_async()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_compute_result)

    def _on_compute_result(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._finish_with_error(f"读取路线规划结果失败：{exc}")
            return

        self._compute_goal_handle = None
        if self._web_task and self._halt_kind:
            return
        if self._cancel_requested or wrapped_result.status == GoalStatus.STATUS_CANCELED:
            self._finish_cancelled()
            return

        result = wrapped_result.result
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._finish_with_error(
                f"Route Server 规划失败，Action 状态为 {wrapped_result.status}。"
            )
            return

        if result.error_code != 0:
            self._finish_with_error(
                f"Route Server 返回错误码 {result.error_code}，未执行路线。"
            )
            return

        path = result.path
        if not path.poses:
            self._finish_with_error("Route Server 返回了空路径。")
            return

        if not path.header.frame_id:
            self._finish_with_error("路线结果没有设置 Path 的 frame_id。")
            return

        if self._cancel_requested:
            self._finish_cancelled()
            return

        try:
            segments = split_route_path(path, result.route.nodes)
        except ValueError as exc:
            self._finish_with_error(f"路线分段失败：{exc}")
            return
        node_ids = [node.nodeid for node in result.route.nodes]
        if self._web_task:
            merged = Path()
            merged.header = deepcopy(path.header)
            for segment in segments:
                merged.poses.extend(deepcopy(segment.poses[1:] if merged.poses else segment.poses))
            segments = [merged]
            node_ids = [node_ids[0], node_ids[-1]]
        if (
            node_ids[0] != self._planning_ids[self._planning_index]
            or node_ids[-1] != self._planning_ids[self._planning_index + 1]
        ):
            self._finish_with_error("路线规划结果的起终点与本次请求不一致。")
            return
        if self._segments:
            if (
                self._segments[0].header.frame_id != path.header.frame_id
                or self._route_node_ids[-1] != node_ids[0]
            ):
                self._finish_with_error("多段规划的坐标系或连接节点不一致，未执行巡检。")
                return
            self._route_node_ids.extend(node_ids[1:])
        else:
            self._route_node_ids = node_ids
        self._segments.extend(segments)
        self._planned_path_points += len(path.poses) - (1 if self._planning_index else 0)
        self._planning_index += 1
        if self._planning_index < len(self._planning_ids) - 1:
            self._submit_compute_leg()
            return

        self._segment_index = 0
        if self._web_task:
            try:
                self._web_ready()
                validate_footprints(self._segments, self._web_task["stations"], self._costmap, self._footprint)
                self._web_ready()
            except (ValueError, TransformException) as error:
                self._finish_with_error(f"路线轮廓校验失败：{error}")
                return
            self._event("planned", "全部路段已规划并通过机器人轮廓校验")
        self._publish_patrol_path()
        self.get_logger().info(
            f"路线已规划：{self._planned_path_points} 个路径点，frame={path.header.frame_id}；"
            f"按 {len(self._segments)} 段依次跟随。"
        )
        unused_ids = set(self._point_tasks) - set(self._route_node_ids)
        if unused_ids:
            self.get_logger().warning(
                f"任务配置中的节点 {sorted(unused_ids)} 不在本次路线中，不执行其任务。"
            )
        self._enter_route()

    def _publish_patrol_path(self):
        """Publish the complete mission for RViz, independently of Nav2's /plan."""
        path = Path()
        path.header.frame_id = (
            self._segments[0].header.frame_id if self._segments else "map"
        )
        path.header.stamp = self.get_clock().now().to_msg()
        for segment in self._segments:
            # Adjacent edges share a boundary pose; keep it once in the display.
            poses = segment.poses[1:] if path.poses else segment.poses
            path.poses.extend(deepcopy(poses))
        for pose in path.poses:
            pose.header = deepcopy(path.header)
        self._path_publisher.publish(path)

    def _publish_status(self):
        goals = self._owned_goals()
        document = {
            "run_id": self._run_id, "state": self._state,
            "cancel_requested": self._cancel_requested,
            "segment_index": self._segment_index,
            "total_segments": len(self._segments),
            "station_node_id": self._station_node_id,
            "station_name": next((station["name"] for station in self._web_task["stations"]
                                  if station["node_id"] == self._station_node_id), None) if self._web_task else None,
            "action_index": self._task_index,
            "action": self._active_task.kind if self._active_task else None,
            "outcome": self._last_outcome,
            "last_summary": self._last_summary,
            "executor_instance": self._executor_instance,
            "task_id": self._last_web_task_id,
            "completed_points": self._completed_points,
            "total_points": len(self._web_task["stations"]) if self._web_task else self._last_summary.get("total_points", 0),
            "events": self._events,
            "event_seq": self._event_seq,
            "active_goals": goals,
            "halt_detail": self._halt_detail,
        }
        self._status_publisher.publish(String(data=json.dumps(document, allow_nan=False)))

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _enter_route(self):
        start_pose = deepcopy(self._segments[0].poses[0])
        try:
            transform = self._tf_buffer.lookup_transform(
                start_pose.header.frame_id, self._base_frame, Time()
            ).transform
        except TransformException as exc:
            self._finish_with_error(f"无法读取机器人位姿，未执行巡检：{exc}")
            return
        distance = math.hypot(
            transform.translation.x - start_pose.pose.position.x,
            transform.translation.y - start_pose.pose.position.y,
        )
        if distance <= self._start_tolerance:
            self._on_start_reached()
            return

        goal = NavigateToPose.Goal()
        start_pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = start_pose
        self.get_logger().info(
            f"距路线起点 {distance:.2f} m，先导航到节点 {self._route_node_ids[0]}。"
        )
        self._send_mission_action(
            self._navigate_client, goal, "entering", self._on_start_reached
        )

    def _on_start_reached(self):
        self.get_logger().info(f"已到达路线起点节点 {self._route_node_ids[0]}。")
        self._begin_station(self._route_node_ids[0], self._send_segment)

    def _on_odometry(self, message):
        twist = message.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        angular = abs(twist.angular.z)
        if math.isfinite(speed) and math.isfinite(angular):
            self._odom_speed = (speed, angular)
            self._odom_received_at = self._now_seconds()
            self._odom_wall = time.monotonic()
            self._odom_stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9

    def _begin_station(self, node_id, after_station):
        self._station_node_id = node_id
        self._station_tasks = self._point_tasks.get(node_id, [])
        self._task_index = 0
        self._aligned = False
        self._after_station = after_station
        self._state = "stopping"
        self._stop_deadline = self._now_seconds() + self._stop_timeout
        self._stopped_since = None
        self.get_logger().info(f"节点 {node_id}：等待车辆停稳。")
        self._event("station", f"到达节点 {node_id}，等待停车")

    def _poll_flow(self):
        if self._web_task and self._state not in {"idle", "finished", "paused"}:
            try:
                transform = self._web_ready()
                if self._active_task and self._active_task.kind == "spin" and self._spin_last_yaw is not None:
                    yaw = yaw_of(transform.rotation)
                    self._spin_progress += math.atan2(math.sin(yaw - self._spin_last_yaw),
                                                     math.cos(yaw - self._spin_last_yaw))
                    self._spin_last_yaw = yaw
            except (ValueError, TransformException) as error:
                if self._halt_kind != "failed":
                    self._request_halt("failed", str(error))
        if self._web_task and self._halt_kind:
            self._poll_halt()
            return
        now = self._now_seconds()
        if self._state == "stopping":
            if self._cancel_requested:
                self._finish_cancelled()
                return
            stopped = (
                self._odom_received_at is not None
                and 0 <= now - self._odom_received_at <= 1.0
                and self._odom_speed[0] <= 0.02
                and self._odom_speed[1] <= 0.02
                and (not self._web_task or (time.monotonic() - self._odom_wall <= 1
                     and 0 <= now - self._odom_stamp <= 1))
            )
            if stopped:
                if self._stopped_since is None:
                    self._stopped_since = now
                if now - self._stopped_since >= self._stop_settle_time:
                    self._stop_deadline = None
                    self._run_next_task()
                    return
            else:
                self._stopped_since = None
            if now >= self._stop_deadline:
                self._finish_with_error(
                    f"节点 {self._station_node_id} 停车确认超时；"
                    "请检查轮速里程计和速度指令，未执行后续任务。"
                )
            return

        if self._mission_deadline is not None and now >= self._mission_deadline:
            self._mission_deadline = None
            self._abort_reason = (
                f"节点 {self._station_node_id} 的第 {self._task_index + 1} 项任务超时。"
            )
            self.get_logger().error(self._abort_reason + " 正在取消任务并等待停止确认。")
            if self._mission_goal_handle is not None:
                self._mission_goal_handle.cancel_goal_async()

    @staticmethod
    def _set_duration(duration, seconds):
        nanoseconds = round(seconds * 1e9)
        duration.sec = nanoseconds // 1_000_000_000
        duration.nanosec = nanoseconds % 1_000_000_000

    def _run_next_task(self):
        if self._cancel_requested:
            self._finish_cancelled()
            return
        if self._web_task and not self._aligned:
            station = next(item for item in self._web_task["stations"] if item["node_id"] == self._station_node_id)
            try:
                transform = self._web_ready()
                delta = math.radians(station["heading"]) - yaw_of(transform.rotation)
                delta = math.atan2(math.sin(delta), math.cos(delta))
            except (ValueError, TransformException) as error:
                self._finish_with_error(str(error))
                return
            if abs(delta) > .05:
                goal = Spin.Goal()
                goal.target_yaw = delta
                self._set_duration(goal.time_allowance, 30)
                self._send_mission_action(self._spin_client, goal, "aligning", self._on_heading_succeeded,
                                          timeout_sec=30)
                self._event("aligning", f"节点 {self._station_node_id} 调整到点朝向")
                return
            self._aligned = True
        if self._task_index >= len(self._station_tasks):
            if self._web_task:
                self._completed_points += 1
                self._event("station_completed", f"节点 {self._station_node_id} 的全部动作已完成")
            after_station = self._after_station
            self._after_station = None
            self._active_task = None
            after_station()
            return

        task = self._station_tasks[self._task_index]
        self._active_task = task
        if self._web_task:
            self._action_started = self._now_seconds()
            self._spin_progress = 0.0
            try:
                self._spin_last_yaw = yaw_of(self._web_ready().rotation) if task.kind == "spin" else None
            except (ValueError, TransformException) as error:
                self._finish_with_error(str(error))
                return
            if (task.kind == "wait" and task.duration_sec <= .001) or (task.kind == "spin" and abs(task.angle_rad) <= .001):
                self._on_task_succeeded()
                return
        self._last_task_feedback_log_time = 0.0
        if task.kind == "spin":
            goal = Spin.Goal()
            goal.target_yaw = task.angle_rad
            self._set_duration(goal.time_allowance, task.timeout_sec)
            client = self._spin_client
            description = f"原地旋转 {math.degrees(task.angle_rad):.1f}°"
        else:
            goal = Wait.Goal()
            self._set_duration(goal.time, task.duration_sec)
            client = self._wait_client
            description = f"等待 {task.duration_sec:.1f} 秒"
        self.get_logger().info(
            f"节点 {self._station_node_id}：开始任务 "
            f"{self._task_index + 1}/{len(self._station_tasks)}，{description}。"
        )
        self._send_mission_action(
            client,
            goal,
            "task",
            self._on_task_succeeded,
            timeout_sec=task.timeout_sec,
            feedback_callback=self._on_task_feedback,
        )
        self._event("action", f"节点 {self._station_node_id} 开始第 {self._task_index + 1} 项：{description}")

    def _on_heading_succeeded(self):
        self._aligned = True
        self._state = "stopping"
        self._stop_deadline = self._now_seconds() + self._stop_timeout
        self._stopped_since = None

    def _send_mission_action(
        self, client, goal, state, on_success, *, timeout_sec=None, feedback_callback=None
    ):
        if self._cancel_requested:
            self._finish_cancelled()
            return
        self._mission_label = (
            "前往路线起点"
            if state == "entering"
            else f"节点 {self._station_node_id} 的第 {self._task_index + 1} 项任务"
        )
        if not client.server_is_ready():
            self._finish_with_error(
                f"{self._mission_label}对应的 Nav2 Action 未就绪；巡检已停止。"
            )
            return
        self._state = state
        self._mission_action_name = "navigate_to_pose" if state == "entering" else "spin" if client is self._spin_client else "wait"
        self._mission_deadline = (
            self._now_seconds() + timeout_sec if timeout_sec is not None else None
        )
        try:
            self._send_pending = True
            future = self._send_goal(client, goal, self._mission_action_name, feedback_callback)
        except Exception as exc:
            self._send_pending = False
            self._finish_with_error(f"提交{self._mission_label}请求失败：{exc}")
            return
        future.add_done_callback(
            lambda completed: self._on_mission_goal_response(completed, on_success)
        )

    def _on_mission_goal_response(self, future, on_success):
        self._send_pending = False
        self._pending_goal = None
        try:
            handle = future.result()
        except Exception as exc:
            self._finish_with_error(f"读取巡检 Action 响应失败：{exc}")
            return
        if not handle.accepted:
            if self._cancel_requested:
                self._finish_cancelled()
            else:
                self._finish_with_error(
                    f"Nav2 拒绝了{self._mission_label}；请确认 Nav2 已激活。"
                )
            return
        self._mission_goal_handle = handle
        self._event("goal_accepted", f"{self._mission_action_name} 已接受")
        if self._cancel_requested or self._abort_reason or self._halt_kind:
            handle.cancel_goal_async()
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_mission_result(completed, on_success)
        )

    def _on_mission_result(self, future, on_success):
        try:
            wrapped = future.result()
        except Exception as exc:
            self._finish_with_error(f"读取巡检 Action 结果失败：{exc}")
            return
        self._mission_goal_handle = None
        self._mission_deadline = None
        if self._web_task and self._halt_kind:
            if wrapped.status not in {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED}:
                self._request_halt("failed", "暂停或取消期间 Nav2 点位动作失败")
            if self._active_task and self._active_task.kind == "wait" and self._action_started is not None:
                self._halt_action_elapsed = self._now_seconds() - self._action_started
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.error_code == 0:
                if self._resume_phase == "task":
                    self._task_index += 1
                    self._active_task = None
                    self._resume_phase = "stopping"
                elif self._resume_phase == "aligning":
                    self._aligned = True
                    self._resume_phase = "stopping"
            return
        if self._abort_reason:
            self._finish_with_error(self._abort_reason)
            return
        if self._cancel_requested or wrapped.status == GoalStatus.STATUS_CANCELED:
            self._finish_cancelled()
            return
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
            self._finish_with_error(
                f"{self._mission_label}执行失败：{result.error_msg or 'Nav2 Action 失败'}"
                f"（错误码 {result.error_code}）；巡检已停止。"
            )
            return
        on_success()

    def _on_task_feedback(self, message):
        now = time.monotonic()
        if now - self._last_task_feedback_log_time < 2.0 or self._active_task is None:
            return
        if self._active_task.kind == "spin":
            progress = math.degrees(abs(message.feedback.angular_distance_traveled))
            target = math.degrees(abs(self._active_task.angle_rad))
            detail = f"旋转进度 {progress:.1f}° / {target:.1f}°"
        else:
            remaining = message.feedback.time_left
            detail = f"剩余等待 {remaining.sec + remaining.nanosec / 1e9:.1f} 秒"
        self.get_logger().info(f"节点 {self._station_node_id}：{detail}。")
        self._last_task_feedback_log_time = now

    def _on_task_succeeded(self):
        self.get_logger().info(
            f"节点 {self._station_node_id}：任务 {self._task_index + 1} 执行完成。"
        )
        self._task_index += 1
        self._active_task = None
        self._event("action_completed", f"节点 {self._station_node_id} 第 {self._task_index} 项动作完成")
        # Wait for the spin command to settle before another task or path goal.
        self._state = "stopping"
        self._stop_deadline = self._now_seconds() + self._stop_timeout
        self._stopped_since = None

    def _send_segment(self):
        if self._cancel_requested:
            self._finish_cancelled()
            return
        if self._web_task:
            try:
                self._web_ready()
                validate_footprints([self._segments[self._segment_index]], [], self._costmap, self._footprint)
                self._web_ready()
            except (ValueError, TransformException) as error:
                self._finish_with_error(str(error))
                return

        goal = FollowPath.Goal()
        goal.path = self._segments[self._segment_index]
        self._active_path_publisher.publish(goal.path)
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id
        goal.progress_checker_id = self._progress_checker_id

        self._state = "following"
        self._last_feedback_log_time = 0.0
        start_id = self._route_node_ids[self._segment_index]
        end_id = self._route_node_ids[self._segment_index + 1]
        self.get_logger().info(
            f"开始第 {self._segment_index + 1}/{len(self._segments)} 段："
            f"节点 {start_id} -> {end_id}；先对齐本段方向，再前进。"
        )
        try:
            self._send_pending = True
            follow_future = self._send_goal(self._follow_client, goal, "follow_path", self._on_follow_feedback)
        except Exception as error:
            self._send_pending = False
            self._finish_with_error(str(error))
            return
        follow_future.add_done_callback(self._on_follow_goal_response)
        self._event("following", f"前往节点 {end_id}")

    def _on_follow_feedback(self, feedback_message):
        now = time.monotonic()
        if now - self._last_feedback_log_time < 2.0:
            return

        feedback = feedback_message.feedback
        self.get_logger().info(
            f"第 {self._segment_index + 1}/{len(self._segments)} 段反馈："
            f"距本段终点 {feedback.distance_to_goal:.2f} m，"
            f"当前线速度 {feedback.speed:.3f} m/s。"
        )
        self._last_feedback_log_time = now

    def _on_follow_goal_response(self, future):
        self._send_pending = False
        self._pending_goal = None
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_with_error(f"提交路径跟随请求失败：{exc}")
            return

        if not goal_handle.accepted:
            self._finish_with_error("Controller Server 拒绝了 FollowPath 请求。")
            return

        self._follow_goal_handle = goal_handle
        self._event("goal_accepted", "路径跟随请求已接受")
        if self._cancel_requested or self._halt_kind:
            goal_handle.cancel_goal_async()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_follow_result)

    def _on_follow_result(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._finish_with_error(f"读取路径跟随结果失败：{exc}")
            return

        self._follow_goal_handle = None
        if self._web_task and self._halt_kind:
            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED and wrapped_result.result.error_code == 0:
                self._resume_phase = "following"
            return
        if self._cancel_requested or wrapped_result.status == GoalStatus.STATUS_CANCELED:
            self._finish_cancelled()
            return

        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped_result.result.error_code != 0
        ):
            result = wrapped_result.result
            error_names = {
                100: "UNKNOWN",
                101: "INVALID_CONTROLLER",
                102: "TF_ERROR",
                103: "INVALID_PATH",
                104: "PATIENCE_EXCEEDED",
                105: "FAILED_TO_MAKE_PROGRESS",
                106: "NO_VALID_CONTROL",
                107: "CONTROLLER_TIMED_OUT",
            }
            error_name = error_names.get(result.error_code, "未知错误")
            detail = result.error_msg or (
                f"{error_name}（错误码 {result.error_code}）"
            )
            self._finish_with_error(
                f"第 {self._segment_index + 1} 段路线跟随失败：{detail}"
            )
            return

        self.get_logger().info(
            f"已到达节点 {self._route_node_ids[self._segment_index + 1]}。"
        )
        self._begin_station(
            self._route_node_ids[self._segment_index + 1], self._advance_segment
        )

    def _advance_segment(self):
        self._segment_index += 1
        if self._segment_index < len(self._segments):
            self._send_segment()
            return

        self._reset_mission("completed")
        self.get_logger().info(
            f"路线执行完成：节点 {self._start_id} -> {self._goal_id}。"
        )

    def _cancel_route(self, _request, response):
        if self._web_task and self._state != "finished":
            self._request_halt("cancelled", "用户请求取消任务")
            response.success = True
            response.message = "正在取消当前动作并确认停车"
            return response
        if self._state == "previewing":
            response.success = False
            response.message = "正在预览路线，等待网关恢复原路线图后再操作。"
            return response
        if self._state == "idle":
            response.success = False
            response.message = "当前没有运行中的路线任务。"
            return response

        self._cancel_requested = True
        self._mission_deadline = None
        active_goal = (
            self._mission_goal_handle or self._follow_goal_handle or self._compute_goal_handle
        )
        if active_goal is not None:
            active_goal.cancel_goal_async()
        elif self._state == "stopping":
            self._finish_cancelled()

        response.success = True
        response.message = "已请求取消路线任务。"
        self.get_logger().info(response.message)
        return response

    def _pause_web(self, _request, response):
        if not self._web_task or self._state in {"finished", "idle"}:
            response.message = "没有可暂停的 Web 任务"
            return response
        if self._state == "paused" or self._halt_kind == "paused":
            response.success = True
            return response
        if self._halt_kind:
            response.message = "正在停止，不能暂停"
            return response
        self._request_halt("paused", "用户请求暂停")
        response.success = True
        response.message = "已请求暂停，正在确认停车"
        return response

    def _request_halt(self, kind, detail):
        if self._state == "finished":
            return
        if self._halt_kind == "failed" and kind != "failed":
            return
        if self._halt_kind is None:
            self._resume_phase = self._state
            self._halt_action_elapsed = None
            self._halt_deadline = time.monotonic() + self._stop_timeout
            self._stopped_since = None
        self._halt_kind = kind
        self._halt_detail = detail
        self._mission_deadline = None
        self._state = "pausing" if kind == "paused" else "cancelling"
        for handle in (self._mission_goal_handle, self._follow_goal_handle, self._compute_goal_handle):
            if handle is not None:
                handle.cancel_goal_async()
        self._event("halt_requested", detail)

    def _poll_halt(self):
        now = time.monotonic()
        active = self._send_pending or any(handle is not None for handle in (
            self._compute_goal_handle, self._follow_goal_handle, self._mission_goal_handle))
        stopped = (not active and self._odom_wall is not None and now - self._odom_wall <= 1
                   and 0 <= self._now_seconds() - self._odom_stamp <= 1
                   and self._odom_speed[0] <= .02 and self._odom_speed[1] <= .02)
        if stopped:
            if self._stopped_since is None:
                self._stopped_since = now
            if now - self._stopped_since >= self._stop_settle_time:
                kind = self._halt_kind
                self._halt_kind = None
                if kind == "paused":
                    if self._resume_phase == "task" and self._active_task is not None:
                        task = self._active_task
                        if task.kind == "wait":
                            elapsed = self._halt_action_elapsed
                            if elapsed is None:
                                elapsed = self._now_seconds() - self._action_started
                            remaining = max(0, task.duration_sec - elapsed)
                            self._station_tasks[self._task_index] = replace(task, duration_sec=remaining)
                        else:
                            direction = 1 if task.angle_rad >= 0 else -1
                            remaining = max(0, abs(task.angle_rad) - max(0, direction * self._spin_progress))
                            self._station_tasks[self._task_index] = replace(task, angle_rad=direction * remaining)
                    self._state = "paused"
                    self._event("paused", "已确认停车，任务暂停")
                else:
                    self._reset_mission(kind, self._halt_detail)
                return
        else:
            self._stopped_since = None
        if now > self._halt_deadline and self._state != "stop_unconfirmed":
            self._state = "stop_unconfirmed"
            self._halt_detail += "；停车未确认，保持执行器锁定"
            self._event("stop_unconfirmed", self._halt_detail)

    def _resume_web(self, _request, response):
        if not self._web_task or self._state != "paused":
            response.message = "任务尚未确认暂停"
            return response
        try:
            transform = self._web_ready()
            if self._segments:
                validate_footprints(self._segments[self._segment_index:], [], self._costmap, self._footprint)
            phase = self._resume_phase
            if phase == "following":
                path = self._segments[self._segment_index]
                closest = min(range(len(path.poses)), key=lambda index: math.hypot(
                    path.poses[index].pose.position.x - transform.translation.x,
                    path.poses[index].pose.position.y - transform.translation.y))
                position = path.poses[closest].pose.position
                if math.hypot(position.x - transform.translation.x, position.y - transform.translation.y) > float(self.get_parameter("resume_tolerance").value):
                    raise ValueError("车辆已偏离待执行路段，请重新制定任务")
                tail = deepcopy(path)
                tail.poses = tail.poses[max(0, min(closest, len(path.poses) - 2)):]
                self._segments[self._segment_index] = tail
        except (ValueError, TransformException) as error:
            response.message = str(error)
            return response
        self._halt_detail = ""
        self._event("resumed", "继续执行未完成步骤")
        if phase == "computing":
            self._state = "computing"
            self._segments = []
            self._route_node_ids = []
            self._planning_index = 0
            self._planned_path_points = 0
            self._submit_compute_leg()
        elif phase == "entering":
            self._enter_route()
        elif phase == "following":
            target = self._segments[self._segment_index].poses[-1].pose.position
            if math.hypot(target.x - transform.translation.x, target.y - transform.translation.y) <= self._start_tolerance:
                self._begin_station(self._route_node_ids[self._segment_index + 1], self._advance_segment)
            else:
                self._send_segment()
        else:
            self._state = "stopping"
            self._stop_deadline = self._now_seconds() + self._stop_timeout
            self._stopped_since = None
        response.success = True
        response.message = "已恢复未完成步骤"
        return response

    def _reset_mission(self, outcome=None, detail=""):
        self._last_outcome = outcome
        self._last_summary = {"total_segments": len(self._segments),
                              "completed_segments": self._segment_index, "detail": detail}
        self._last_summary.update(total_points=len(self._web_task["stations"]) if self._web_task else 0,
                                  completed_points=self._completed_points)
        empty_path = Path()
        empty_path.header.frame_id = "map"
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self._active_path_publisher.publish(empty_path)
        self._state = "finished" if self._web_task else "idle"
        self._halt_kind = None
        self._cancel_requested = False
        self._compute_goal_handle = None
        self._follow_goal_handle = None
        self._segments = []
        self._route_node_ids = []
        self._planning_ids = []
        self._planning_index = 0
        self._planned_path_points = 0
        self._mission_goal_handle = None
        self._mission_deadline = None
        self._abort_reason = None
        self._mission_label = ""
        self._station_node_id = None
        self._station_tasks = []
        self._active_task = None
        self._after_station = None
        self._stop_deadline = None
        self._stopped_since = None
        self._event(outcome or "ended", detail or {"completed": "任务完成并已停车", "cancelled": "任务已取消并已停车"}.get(outcome, "任务结束"))

    def _finish_with_error(self, message):
        if self._web_task:
            self._request_halt("failed", message)
            return
        self._reset_mission("failed", message)
        self.get_logger().error(message)

    def _finish_cancelled(self):
        if self._web_task:
            self._request_halt("cancelled", "Nav2 动作已取消，等待停车确认")
            return
        self._reset_mission("cancelled")
        self.get_logger().info("路线任务已取消。")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoutePatrol()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
