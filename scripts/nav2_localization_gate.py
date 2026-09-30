#!/usr/bin/env python3

"""Activate Nav2 after the map, SuperSLAM pose, and TF chain are ready."""

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from super_slam.msg import LocalizationState
from tf2_ros import Buffer, TransformListener


class Nav2LocalizationGate(Node):
    """Wait for localization readiness, then start Nav2's lifecycle manager."""

    def __init__(self) -> None:
        super().__init__("nav2_localization_gate")

        self._localization_valid = False
        self._localization_state = "unknown"
        self._map_ready = False
        self._request_pending = False
        self._navigation_started = False
        self._startup_failed = False
        self._last_wait_message = ""

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            LocalizationState,
            "/lio/localization_state",
            self._on_localization_state,
            10,
        )
        self._poll_timer = self.create_timer(0.5, self._poll)

        self.get_logger().info(
            "Waiting for /map, valid /lio/localization_state, and "
            "map -> base_footprint TF before starting Nav2"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_ready = msg.info.width > 0 and msg.info.height > 0 and bool(msg.data)

    def _on_localization_state(self, msg: LocalizationState) -> None:
        self._localization_valid = msg.valid
        self._localization_state = msg.state_name

    def _poll(self) -> None:
        if self._navigation_started or self._request_pending or self._startup_failed:
            return

        if not self._map_ready:
            self._log_wait("Waiting for a valid OccupancyGrid on /map")
            return

        if not self._localization_valid:
            self._log_wait(
                "Waiting for SuperSLAM localization to become valid "
                f"(state: {self._localization_state})"
            )
            return

        if not self._tf_buffer.can_transform(
            "map",
            "base_footprint",
            Time(),
        ):
            self._log_wait("Waiting for TF map -> base_footprint")
            return

        if not self._lifecycle_client.service_is_ready():
            self._log_wait("Waiting for Nav2 lifecycle manager service")
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self._request_pending = True
        future = self._lifecycle_client.call_async(request)
        future.add_done_callback(self._on_startup_response)

    def _log_wait(self, message: str) -> None:
        if message != self._last_wait_message:
            self.get_logger().info(message)
            self._last_wait_message = message

    def _on_startup_response(self, future) -> None:
        self._request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._startup_failed = True
            self.get_logger().error(f"Nav2 lifecycle startup request failed: {exc}")
            self._poll_timer.cancel()
            return

        if not response.success:
            self._startup_failed = True
            self.get_logger().error(
                "Nav2 lifecycle manager reported startup failure; "
                "inspect the Nav2 node logs and activate it manually from RViz if needed"
            )
            self._poll_timer.cancel()
            return

        self._navigation_started = True
        self.get_logger().info("Nav2 started after map and SuperSLAM readiness checks")
        self._poll_timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2LocalizationGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
