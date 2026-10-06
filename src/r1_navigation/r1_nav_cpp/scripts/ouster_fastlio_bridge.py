#!/usr/bin/env python3
"""
Ouster → FAST-LIO2 compatibility bridge (single node, single process).

Fixes two incompatibilities between ouster-ros and FAST-LIO2:

  1. IMU QoS mismatch
       /ouster/imu          (BEST_EFFORT) → /ouster/imu_reliable  (RELIABLE)
       ouster-ros publishes SensorDataQoS; FAST-LIO subscribes with default RELIABLE.

  2. PointCloud ring field type mismatch
       /ouster/points       ring=uint16   → /ouster/points_fastlio ring=uint8
       ouster-ros publishes ring as uint16 (os_point.h); FAST-LIO preprocess.h
       declares ouster_ros::Point.ring as uint8. pcl::fromROSMsg matches by name
       AND datatype, so it fails silently: "Failed to find match for field 'ring'".
       Fix: patch the PointField datatype descriptor only — no data repacking needed
       because pcl addresses each field by its own offset independently. The low byte
       at offset 26 (little-endian) is already the correct uint8 ring value (0-63).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField


_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class OusterFastlioBridge(Node):
    def __init__(self):
        super().__init__("ouster_fastlio_bridge")

        # --- IMU bridge ---
        self._imu_pub = self.create_publisher(Imu, "/ouster/imu_reliable", _RELIABLE)
        self._imu_sub = self.create_subscription(
            Imu, "/ouster/imu", self._imu_cb, _BEST_EFFORT
        )

        # --- PointCloud bridge ---
        self._pc_pub = self.create_publisher(PointCloud2, "/ouster/points_fastlio", _BEST_EFFORT)
        self._pc_sub = self.create_subscription(
            PointCloud2, "/ouster/points", self._pc_cb, _BEST_EFFORT
        )
        self._ring_patched = False

        self.get_logger().info(
            "ouster_fastlio_bridge ready\n"
            "  /ouster/imu        (BEST_EFFORT) → /ouster/imu_reliable  (RELIABLE)\n"
            "  /ouster/points     ring=uint16   → /ouster/points_fastlio ring=uint8"
        )

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_pub.publish(msg)

    def _pc_cb(self, msg: PointCloud2) -> None:
        for field in msg.fields:
            if field.name == "ring" and field.datatype == PointField.UINT16:
                field.datatype = PointField.UINT8
                if not self._ring_patched:
                    self.get_logger().info("Patched ring field: UINT16 → UINT8")
                    self._ring_patched = True
                break
        self._pc_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OusterFastlioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
