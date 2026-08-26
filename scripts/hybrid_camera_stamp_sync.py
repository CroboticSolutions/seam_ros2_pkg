#!/usr/bin/env python3
"""Republish RealSense topics with sim-time stamps (hybrid Gazebo + real D435).

Driver keeps wall time on /camera_hw/camera/...
This node publishes /camera/camera/... stamped from /clock.

Pointclouds are large; subscribe RELIABLE (matches realsense2_camera) and restamp
in-place on a multithreaded executor so image callbacks are not starved.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def _qos(depth: int = 5) -> QoSProfile:
    # realsense2_camera pointcloud/color use RELIABLE KEEP_LAST.
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class HybridCameraStampSync(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_camera_stamp_sync")
        self.declare_parameter("input_namespace", "/camera_hw/camera")
        self.declare_parameter("output_namespace", "/camera/camera")
        inn = (
            self.get_parameter("input_namespace")
            .get_parameter_value()
            .string_value.rstrip("/")
        )
        out = (
            self.get_parameter("output_namespace")
            .get_parameter_value()
            .string_value.rstrip("/")
        )

        if not self.get_parameter("use_sim_time").get_parameter_value().bool_value:
            self.get_logger().error(
                "use_sim_time is false — hybrid stamp sync will NOT fix TF lookups."
            )

        qos = _qos(5)
        qos_points = _qos(2)

        self._pub_color = self.create_publisher(Image, f"{out}/color/image_raw", qos)
        self._pub_color_info = self.create_publisher(
            CameraInfo, f"{out}/color/camera_info", qos
        )
        self._pub_aligned_depth = self.create_publisher(
            Image, f"{out}/aligned_depth_to_color/image_raw", qos
        )
        self._pub_points = self.create_publisher(
            PointCloud2, f"{out}/depth/color/points", qos_points
        )

        # Mutually exclusive callbacks so a heavy PCL publish cannot overlap itself.
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

        self._img_group = ReentrantCallbackGroup()
        self._pcl_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Image, f"{inn}/color/image_raw", self._on_color, qos,
            callback_group=self._img_group,
        )
        self.create_subscription(
            CameraInfo, f"{inn}/color/camera_info", self._on_color_info, qos,
            callback_group=self._img_group,
        )
        self.create_subscription(
            Image,
            f"{inn}/aligned_depth_to_color/image_raw",
            self._on_aligned_depth,
            qos,
            callback_group=self._img_group,
        )
        self.create_subscription(
            PointCloud2,
            f"{inn}/depth/color/points",
            self._on_points,
            qos_points,
            callback_group=self._pcl_group,
        )

        self._logged_points = False
        self._points_count = 0
        self.get_logger().info(
            f"Hybrid stamp sync: {inn}/… → {out}/… "
            f"(use_sim_time={self.get_parameter('use_sim_time').value}, "
            f"QoS=RELIABLE)"
        )

    def _restamp(self, msg):
        # In-place: avoid copying multi‑MB pointclouds.
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _on_color(self, msg: Image) -> None:
        self._pub_color.publish(self._restamp(msg))

    def _on_color_info(self, msg: CameraInfo) -> None:
        self._pub_color_info.publish(self._restamp(msg))

    def _on_aligned_depth(self, msg: Image) -> None:
        self._pub_aligned_depth.publish(self._restamp(msg))

    def _on_points(self, msg: PointCloud2) -> None:
        out = self._restamp(msg)
        self._points_count += 1
        if not self._logged_points:
            self._logged_points = True
            sim = out.header.stamp.sec + out.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"First pointcloud restamp: sim_stamp={sim:.3f}s "
                f"frame={out.header.frame_id!r} "
                f"size={out.width}x{out.height}"
            )
        self._pub_points.publish(out)


def main() -> None:
    rclpy.init()
    node = HybridCameraStampSync()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
