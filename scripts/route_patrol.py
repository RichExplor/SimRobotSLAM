#!/usr/bin/env python3

"""Execute a route graph path with Nav2's Route Server and controller."""

import math
import time
from collections.abc import Sequence
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path as FilePath

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputeRoute, FollowPath, NavigateToPose, Spin, Wait
from nav2_msgs.msg import RouteNode
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


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
        self._flow_timer = self.create_timer(0.1, self._poll_flow)

        self.create_service(Trigger, "/route_patrol/start", self._start_route)
        self.create_service(Trigger, "/route_patrol/cancel", self._cancel_route)
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
            future = self._compute_client.send_goal_async(goal)
        except Exception as exc:
            self._finish_with_error(f"提交路线规划请求失败：{exc}")
            return False
        future.add_done_callback(self._on_compute_goal_response)
        return True

    def _on_compute_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # ROS futures may report transport errors here.
            self._finish_with_error(f"路线规划请求异常：{exc}")
            return

        if not goal_handle.accepted:
            self._finish_with_error("Route Server 拒绝了路线规划请求。")
            return

        self._compute_goal_handle = goal_handle
        if self._cancel_requested:
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

    def _begin_station(self, node_id, after_station):
        self._station_node_id = node_id
        self._station_tasks = self._point_tasks.get(node_id, [])
        self._task_index = 0
        self._after_station = after_station
        self._state = "stopping"
        self._stop_deadline = self._now_seconds() + self._stop_timeout
        self._stopped_since = None
        self.get_logger().info(f"节点 {node_id}：等待车辆停稳。")

    def _poll_flow(self):
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
        if self._task_index >= len(self._station_tasks):
            after_station = self._after_station
            self._after_station = None
            self._active_task = None
            after_station()
            return

        task = self._station_tasks[self._task_index]
        self._active_task = task
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
        self._mission_deadline = (
            self._now_seconds() + timeout_sec if timeout_sec is not None else None
        )
        try:
            future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        except Exception as exc:
            self._finish_with_error(f"提交{self._mission_label}请求失败：{exc}")
            return
        future.add_done_callback(
            lambda completed: self._on_mission_goal_response(completed, on_success)
        )

    def _on_mission_goal_response(self, future, on_success):
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
        if self._cancel_requested or self._abort_reason:
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
        # Wait for the spin command to settle before another task or path goal.
        self._state = "stopping"
        self._stop_deadline = self._now_seconds() + self._stop_timeout
        self._stopped_since = None

    def _send_segment(self):
        if self._cancel_requested:
            self._finish_cancelled()
            return

        goal = FollowPath.Goal()
        goal.path = self._segments[self._segment_index]
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
        follow_future = self._follow_client.send_goal_async(
            goal, feedback_callback=self._on_follow_feedback
        )
        follow_future.add_done_callback(self._on_follow_goal_response)

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
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_with_error(f"提交路径跟随请求失败：{exc}")
            return

        if not goal_handle.accepted:
            self._finish_with_error("Controller Server 拒绝了 FollowPath 请求。")
            return

        self._follow_goal_handle = goal_handle
        if self._cancel_requested:
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

        self._reset_mission()
        self.get_logger().info(
            f"路线执行完成：节点 {self._start_id} -> {self._goal_id}。"
        )

    def _cancel_route(self, _request, response):
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

    def _reset_mission(self):
        self._state = "idle"
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

    def _finish_with_error(self, message):
        self._reset_mission()
        self.get_logger().error(message)

    def _finish_cancelled(self):
        self._reset_mission()
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
