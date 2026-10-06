#!/usr/bin/env python3
"""Publish AprilTag map poses as RViz markers for visual verification.

This node parses the same tag config the C++ ``apriltag_pose_corrector``
loads and republishes every resolved tag pose as a
``visualization_msgs/MarkerArray`` in the map frame. Overlaying these markers
on the static ``.pgm`` map (the costmap StaticLayer / ``/map``) in RViz is the
quickest way to catch a mis-placed tag: one sitting off an aisle, at the wrong
height, or facing into the shelf instead of out toward the robot.

Both tag config formats are supported, auto-detected by top-level key and
mirroring the relayer's own selection (raw-pose preferred when present):
  * raw-pose ``apriltag_poses.yaml`` (``apriltags`` map of id -> x/y/z/yaw_deg),
    drawn with purple plates; and
  * segment-based ``warehouse.yaml`` (``warehouse`` rows/segments/bays),
    drawn with green (start) / blue (end) plates.

The geometry MUST stay bit-for-bit consistent with
``loadTagPosesFromWarehouseYaml()`` in
``src/apriltag_pose_corrector.cpp``:

  * z of every tag == the row's ``shelf_heights_meters['B']``.
  * xy == ``start_point + distance_along_segment * unit(start -> end)``.
  * orientation basis columns are +X = (-sin, cos, 0), +Y = (0, 0, 1) world up,
    +Z = (cos, sin, 0) aisle normal (the tag's facing direction).
  * ``start_apriltag_id`` always wins over an ``end_apriltag_id`` with the same
    id; a start id is registered unconditionally (later duplicates overwrite).

Per marker id (== apriltag id) three markers are published:
  * a thin CUBE plate oriented in the tag plane (normal == facing +Z),
  * an ARROW from the tag origin along +Z showing the facing direction,
  * a TEXT label with the tag id.

Markers are latched (transient_local) and also periodically re-published so a
late-joining RViz always receives them.

Run:
    ros2 run r1_nav_cpp publish_apriltag_markers.py
    # then add a MarkerArray display on /apriltag_map_markers in RViz.

Params (override with -p name:=value):
    warehouse_yaml   Explicit path to a tag config (either format; the arg name
                     is kept for compatibility). Empty -> resolve from the
                     LOCATION/LIDAR env vars, preferring apriltag_poses.yaml
                     when it exists (matches the relayer).
    map_frame        Frame the markers are published in (default "map").
    marker_size      Tag plate edge length in meters (default 0.20).
    normal_length    Facing-arrow length in meters (default 0.50).
    publish_period_s Re-publish period in seconds (default 2.0).
    text_z_offset    Height of the id label above the tag (default 0.25).
    rotate_about_origin_deg  DEBUG ONLY: rotate every marker pose about the map
                     origin by this angle (default 0.0 = faithful overlay).
    translate_x_m    DEBUG ONLY: translate every marker pose in x after the
                     rotation (default 0.0).
    translate_y_m    DEBUG ONLY: translate every marker pose in y after the
                     rotation (default 0.0).
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:
    # Installed alongside the r1_nav_cpp python package.
    from r1_nav_cpp.config_resolver import (
        get_apriltag_poses_config_path,
        get_warehouse_config_path,
    )
except ImportError:  # pragma: no cover - fallback if run outside install space
    get_warehouse_config_path = None
    get_apriltag_poses_config_path = None

import yaml
from ament_index_python.packages import get_package_share_directory


def _matrix_to_quaternion(r):
    """Convert a 3x3 row-major rotation matrix to (x, y, z, w).

    ``r`` is indexed r[row][col]. Standard Shoemake conversion.
    """
    r00, r01, r02 = r[0]
    r10, r11, r12 = r[1]
    r20, r21, r22 = r[2]
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s
    return x, y, z, w


class TagPose:
    """A resolved tag pose in the map frame."""

    __slots__ = ("tag_id", "x", "y", "z", "yaw_rad", "kind", "row", "bay")

    def __init__(self, tag_id, x, y, z, yaw_rad, kind, row, bay):
        self.tag_id = tag_id
        self.x = x
        self.y = y
        self.z = z
        self.yaw_rad = yaw_rad
        self.kind = kind  # "start" or "end"
        self.row = row
        self.bay = bay

    def quaternion(self, yaw_rad=None):
        yaw = self.yaw_rad if yaw_rad is None else yaw_rad
        s, c = math.sin(yaw), math.cos(yaw)
        # Columns: +X=(-sin,cos,0), +Y=(0,0,1), +Z=(cos,sin,0). Row-major here.
        r = [
            [-s, 0.0, c],
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
        ]
        return _matrix_to_quaternion(r)

    def facing(self, yaw_rad=None):
        """Unit +Z (aisle normal / facing direction) in the map frame."""
        yaw = self.yaw_rad if yaw_rad is None else yaw_rad
        return math.cos(yaw), math.sin(yaw), 0.0


def parse_warehouse_tags(yaml_path, logger=None):
    """Parse warehouse.yaml into {tag_id: TagPose}, mirroring the C++ loader.

    Returns (tags, warnings). ``warnings`` is a list of human-readable strings
    describing skipped/overwritten entries (the lint aspect of the check).
    """
    warnings = []

    def warn(msg):
        warnings.append(msg)
        if logger is not None:
            logger.warning(msg)

    with open(yaml_path, "r") as f:
        root = yaml.safe_load(f)

    tags = {}
    start_tag_ids = set()

    warehouse = (root or {}).get("warehouse", {})
    rows = warehouse.get("rows", {})
    if not isinstance(rows, dict):
        warn(f"warehouse.yaml at '{yaml_path}' has no 'warehouse.rows' map")
        return tags, warnings

    for row_name, row_node in rows.items():
        if not isinstance(row_node, dict):
            continue
        heights = row_node.get("shelf_heights_meters", {})
        have_shelf_b = isinstance(heights, dict) and "B" in heights
        shelf_b_z = float(heights["B"]) if have_shelf_b else 0.0

        segments = row_node.get("segments", [])
        if not isinstance(segments, list):
            continue

        for seg in segments:
            if not isinstance(seg, dict):
                continue
            yaw_rad = float(seg.get("yaw_deg", 0.0)) * math.pi / 180.0
            sp = seg.get("start_point", {}) or {}
            ep = seg.get("end_point", {}) or {}
            sx = float(sp.get("x", 0.0))
            sy = float(sp.get("y", 0.0))
            ex = float(ep.get("x", 0.0))
            ey = float(ep.get("y", 0.0))
            dx, dy = ex - sx, ey - sy
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-6:
                continue
            ux, uy = dx / seg_len, dy / seg_len

            def make_pose(dist, tag_id, kind, bay_name):
                return TagPose(
                    tag_id,
                    sx + dist * ux,
                    sy + dist * uy,
                    shelf_b_z,
                    yaw_rad,
                    kind,
                    row_name,
                    bay_name,
                )

            bays = seg.get("bays", {})
            if not isinstance(bays, dict):
                continue

            for bay_name, bay in bays.items():
                if not isinstance(bay, dict):
                    continue
                # Schema rename: prefer start_apriltag_id, fall back to legacy
                # apriltag_id.
                start_id = -1
                if "start_apriltag_id" in bay:
                    start_id = int(bay.get("start_apriltag_id", -1))
                elif "apriltag_id" in bay:
                    start_id = int(bay.get("apriltag_id", -1))
                end_id = int(bay.get("end_apriltag_id", -1)) if "end_apriltag_id" in bay else -1

                if start_id < 0 and end_id < 0:
                    continue
                if not have_shelf_b:
                    warn(
                        f"row='{row_name}' bay='{bay_name}': skipped, no "
                        f"shelf_heights_meters['B']"
                    )
                    continue

                # Start tag: always wins, overwrites any prior entry.
                if start_id >= 0:
                    start_d = float(bay.get("start_distance", 0.0))
                    if start_id in tags:
                        prev = tags[start_id]
                        warn(
                            f"duplicate start_apriltag_id={start_id}: "
                            f"row='{prev.row}' bay='{prev.bay}' "
                            f"({prev.x:.3f}, {prev.y:.3f}) overwritten by "
                            f"row='{row_name}' bay='{bay_name}' "
                            f"({sx + start_d * ux:.3f}, {sy + start_d * uy:.3f})"
                        )
                    tags[start_id] = make_pose(start_d, start_id, "start", bay_name)
                    start_tag_ids.add(start_id)

                # End tag: only when no start tag shares this id.
                if end_id >= 0:
                    if end_id == start_id or end_id in start_tag_ids:
                        warn(
                            f"end_apriltag_id={end_id} (row='{row_name}' "
                            f"bay='{bay_name}') ignored: start id takes priority"
                        )
                    elif "end_distance" not in bay:
                        warn(
                            f"end_apriltag_id={end_id} (row='{row_name}' "
                            f"bay='{bay_name}') skipped: no end_distance"
                        )
                    elif end_id in tags:
                        warn(
                            f"duplicate end_apriltag_id={end_id} (row='{row_name}' "
                            f"bay='{bay_name}'); keeping first registered pose"
                        )
                    else:
                        end_d = float(bay.get("end_distance", 0.0))
                        tags[end_id] = make_pose(end_d, end_id, "end", bay_name)

    return tags, warnings


def parse_apriltag_poses(yaml_path, logger=None):
    """Parse the raw-pose apriltag_poses.yaml into {tag_id: TagPose}.

    Mirrors the relayer's loadTagPosesFromPosesYaml(): an ``apriltags`` map
    keyed by (stringified) tag id, each with x/y (required) and optional
    z/yaw_deg. Returns (tags, warnings).
    """
    warnings = []

    def warn(msg):
        warnings.append(msg)
        if logger is not None:
            logger.warning(msg)

    with open(yaml_path, "r") as f:
        root = yaml.safe_load(f)

    tags = {}
    entries = (root or {}).get("apriltags", {})
    if not isinstance(entries, dict):
        warn(f"apriltag_poses.yaml at '{yaml_path}' has no 'apriltags' map")
        return tags, warnings

    for key, node in entries.items():
        try:
            tag_id = int(key)
        except (ValueError, TypeError):
            warn(f"apriltag_poses.yaml: non-integer tag key '{key}' skipped")
            continue
        if not isinstance(node, dict) or "x" not in node or "y" not in node:
            warn(f"apriltag_poses.yaml: tag '{key}' missing x/y; skipped")
            continue
        x = float(node["x"])
        y = float(node["y"])
        z = float(node.get("z", 0.0))
        yaw_rad = math.radians(float(node.get("yaw_deg", 0.0)))
        tags[tag_id] = TagPose(
            tag_id, x, y, z, yaw_rad, "raw", "apriltag_poses", str(key))

    return tags, warnings


def load_tag_poses(yaml_path, logger=None):
    """Load tag poses from either format, auto-detecting by top-level keys.

    Returns (tags, warnings, fmt) where fmt is "raw-pose" or "segment". A file
    with a top-level ``apriltags`` key is treated as the raw-pose format; one
    with ``warehouse`` as the segment format. This mirrors the relayer's
    selection so the overlay matches whatever the relayer would load.
    """
    with open(yaml_path, "r") as f:
        root = yaml.safe_load(f)

    if isinstance(root, dict) and "apriltags" in root:
        tags, warnings = parse_apriltag_poses(yaml_path, logger)
        return tags, warnings, "raw-pose"
    tags, warnings = parse_warehouse_tags(yaml_path, logger)
    return tags, warnings, "segment"


class AprilTagMarkerPublisher(Node):
    def __init__(self):
        super().__init__("apriltag_marker_publisher")

        default_yaml = self.declare_parameter("warehouse_yaml", "").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.marker_size = float(self.declare_parameter("marker_size", 0.20).value)
        self.normal_length = float(self.declare_parameter("normal_length", 0.50).value)
        self.text_z_offset = float(self.declare_parameter("text_z_offset", 0.25).value)
        publish_period_s = float(self.declare_parameter("publish_period_s", 2.0).value)
        # DEBUG/VERIFICATION ONLY. Apply a rigid 2D transform to every published
        # marker pose about the map origin: first rotate (x, y) by
        # rotate_about_origin_deg about (0, 0) and add that angle to yaw, then
        # translate by (translate_x_m, translate_y_m):
        #   (x, y) -> R(theta) * (x, y) + (tx, ty),  yaw -> yaw + theta.
        # All-zero (the default) keeps the markers faithful to the relayer's
        # tag_map_pose_ (positions used verbatim in the xy residual). Set
        # rotate_about_origin_deg=90.0 to test the "tags appear rotated -90 deg
        # from true" report, and/or a translation to nudge the whole overlay.
        # This does NOT affect the relayer/correction path -- it only changes
        # what this visualization topic publishes.
        self.rotate_about_origin_deg = float(
            self.declare_parameter("rotate_about_origin_deg", 0.0).value)
        self.rotate_rad = math.radians(self.rotate_about_origin_deg)
        self.translate_x_m = float(
            self.declare_parameter("translate_x_m", 0.0).value)
        self.translate_y_m = float(
            self.declare_parameter("translate_y_m", 0.0).value)
        # Always surface the resolved values so it's obvious at a glance whether
        # the overrides actually reached the node.
        self.get_logger().info(
            f"rotate_about_origin_deg = {self.rotate_about_origin_deg:.3f}, "
            f"translate_x_m = {self.translate_x_m:.3f}, "
            f"translate_y_m = {self.translate_y_m:.3f} "
            f"(all 0.0 => faithful overlay)")

        yaml_path = default_yaml or self._resolve_default_yaml()
        self.get_logger().info(f"Loading AprilTag map poses from: {yaml_path}")

        try:
            self.tags, warnings, fmt = load_tag_poses(yaml_path, self.get_logger())
        except Exception as exc:  # noqa: BLE001 - surface any parse failure
            self.get_logger().error(f"Failed to parse '{yaml_path}': {exc}")
            self.tags, warnings, fmt = {}, [], "unknown"

        self.get_logger().info(
            f"Loaded {len(self.tags)} AprilTag pose(s) [{fmt} format]; "
            f"{len(warnings)} warning(s)."
        )
        if (self.rotate_about_origin_deg != 0.0
                or self.translate_x_m != 0.0 or self.translate_y_m != 0.0):
            self.get_logger().warning(
                f"rotate_about_origin_deg={self.rotate_about_origin_deg:.1f}, "
                f"translate=({self.translate_x_m:.3f}, {self.translate_y_m:.3f}): "
                "published marker poses are transformed about the map origin "
                "for VERIFICATION ONLY; they no longer match the relayer's tag "
                "poses. Set all back to 0.0 for the faithful overlay."
            )
        for tag_id in sorted(self.tags):
            t = self.tags[tag_id]
            self.get_logger().info(
                f"  id={tag_id} ({t.kind}) row='{t.row}' bay='{t.bay}' "
                f"map=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) "
                f"facing yaw={math.degrees(t.yaw_rad):.1f} deg"
            )

        # Latched so a late-joining RViz still receives the last array.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(MarkerArray, "apriltag_map_markers", qos)

        self.marker_array = self._build_marker_array()
        self.publish_once()
        self.timer = self.create_timer(max(0.1, publish_period_s), self.publish_once)

    def _resolve_default_yaml(self):
        pkg_share = get_package_share_directory("r1_nav_cpp")
        if get_warehouse_config_path is not None:
            # Prefer the refined raw-pose file when present, exactly like the
            # relayer's loadTagPoses() selection.
            if get_apriltag_poses_config_path is not None:
                poses_path = get_apriltag_poses_config_path(pkg_share)
                if os.path.isfile(poses_path):
                    return poses_path
            return get_warehouse_config_path(pkg_share)
        # Fallback replicating config_resolver if the package import failed.
        loc = os.environ.get("LOCATION", "YONDU").lower()
        lidar = os.environ.get("LIDAR", "LIVOX").lower()
        base = os.path.join(pkg_share, "config", "warehouse_configs", loc, lidar)
        poses_path = os.path.join(base, "apriltag_poses.yaml")
        if os.path.isfile(poses_path):
            return poses_path
        return os.path.join(base, "warehouse.yaml")

    def _rotate_about_origin(self, x, y, yaw):
        """Apply the debug transform about the map origin.

        Rotates (x, y) by rotate_rad about (0, 0), adds rotate_rad to yaw, then
        translates by (translate_x_m, translate_y_m). No-op when rotation and
        translation are all zero. Verification aid only (see the
        rotate_about_origin_deg / translate_x_m / translate_y_m parameters).
        """
        if (self.rotate_rad == 0.0
                and self.translate_x_m == 0.0 and self.translate_y_m == 0.0):
            return x, y, yaw
        c, s = math.cos(self.rotate_rad), math.sin(self.rotate_rad)
        rx = c * x - s * y + self.translate_x_m
        ry = s * x + c * y + self.translate_y_m
        return rx, ry, yaw + self.rotate_rad

    def _build_marker_array(self):
        arr = MarkerArray()
        # Clear any stale markers from a previous run first.
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        # Plate color by tag kind: start=green, end=blue, raw-pose=purple.
        kind_colors = {
            "start": ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.9),
            "end": ColorRGBA(r=0.1, g=0.5, b=0.9, a=0.9),
            "raw": ColorRGBA(r=0.7, g=0.2, b=0.9, a=0.9),
        }
        for tag_id in sorted(self.tags):
            t = self.tags[tag_id]
            color = kind_colors.get(t.kind, ColorRGBA(r=0.1, g=0.5, b=0.9, a=0.9))
            # Apply the optional debug rotation about the map origin. px/py/pyaw
            # are what actually gets published; t.x/t.y/t.yaw_rad stay as the
            # true (relayer-consistent) values.
            px, py, pyaw = self._rotate_about_origin(t.x, t.y, t.yaw_rad)
            qx, qy, qz, qw = t.quaternion(pyaw)

            # Thin plate in the tag plane (normal == facing +Z).
            plate = Marker()
            plate.header.frame_id = self.map_frame
            plate.ns = "apriltag_plate"
            plate.id = tag_id
            plate.type = Marker.CUBE
            plate.action = Marker.ADD
            plate.pose.position.x = px
            plate.pose.position.y = py
            plate.pose.position.z = t.z
            plate.pose.orientation.x = qx
            plate.pose.orientation.y = qy
            plate.pose.orientation.z = qz
            plate.pose.orientation.w = qw
            plate.scale.x = self.marker_size
            plate.scale.y = self.marker_size
            plate.scale.z = 0.02
            plate.color = color
            arr.markers.append(plate)

            # Facing arrow from tag origin along +Z.
            fx, fy, fz = t.facing(pyaw)
            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.ns = "apriltag_facing"
            arrow.id = tag_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=px, y=py, z=t.z),
                Point(
                    x=px + self.normal_length * fx,
                    y=py + self.normal_length * fy,
                    z=t.z + self.normal_length * fz,
                ),
            ]
            arrow.scale.x = 0.03  # shaft diameter
            arrow.scale.y = 0.06  # head diameter
            arrow.scale.z = 0.08  # head length
            arrow.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9)
            arr.markers.append(arrow)

            # Id label.
            text = Marker()
            text.header.frame_id = self.map_frame
            text.ns = "apriltag_label"
            text.id = tag_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = px
            text.pose.position.y = py
            text.pose.position.z = t.z + self.text_z_offset
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = str(tag_id)
            arr.markers.append(text)

        return arr

    def publish_once(self):
        # Re-stamp on each publish so RViz TF timing is happy.
        now = self.get_clock().now().to_msg()
        for m in self.marker_array.markers:
            m.header.stamp = now
        self.pub.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
