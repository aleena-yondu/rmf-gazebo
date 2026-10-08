#!/usr/bin/env python3
"""Lock map->odom yaw from the wall angles, then fit the translation.

Gazebo wall vertices are rotated onto yondu_simmap. The score is the
distance-transform cost: how far those vertices sit from occupied pixels.

    python3 src/align_map_to_gazebo.py

goal_sim_ground_truth.launch.py calls align() and publishes the result.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

_PACKAGE_ROOT = Path(__file__).resolve().parent.parent
MAP_YAML = _PACKAGE_ROOT / 'maps' / 'yondu_simmap.yaml'
WALL_OBJ = (
    _PACKAGE_ROOT / 'models' / 'yondu_warehouse_yondu' / 'meshes' / 'wall_1.obj'
)
# Link pose of wall_1 in models/yondu_warehouse_yondu/model.sdf.
WALL_POSE_XY = (5.744842791026957, -15.83881460376728)
# Spawn yaw is 0, so the robot faces gazebo +X and its left is +Y.
# The wall centroid sits against the right-hand corridor wall.
LEFT_OFFSET_M = 1.0


@dataclass
class Alignment:
    spawn_x: float
    spawn_y: float
    spawn_z: float
    map_x: float
    map_y: float
    map_z: float
    map_yaw: float
    report: str


def _image_path(config: dict) -> Path:
    image = Path(config['image'])
    if image.is_absolute():
        return image
    return MAP_YAML.parent / image


def _gazebo_points() -> np.ndarray:
    points = []
    for line in WALL_OBJ.read_text().splitlines():
        if not line.startswith('v '):
            continue
        _tag, x, y, _z = line.split()[:4]
        points.append((
            float(x) + WALL_POSE_XY[0],
            float(y) + WALL_POSE_XY[1],
        ))
    return np.asarray(points, dtype=float)


def _rect_angle_deg(points_xy: np.ndarray) -> float:
    """Long-edge angle of a point set, in degrees, y-up, range [-90, 90)."""
    rect = cv2.minAreaRect(points_xy.astype(np.float32))
    (width, height), angle = rect[1], rect[2]
    # OpenCV's angle is the rotation of the width edge, in [-90, 0).
    if width < height:
        angle += 90.0
    if angle >= 90.0:
        angle -= 180.0
    return float(angle)


def _rotate(points: np.ndarray, yaw: float) -> np.ndarray:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T


def _cost(rotated, translation, dist_map, origin, resolution, height):
    test = rotated + translation
    px = np.rint((test[:, 0] - origin[0]) / resolution).astype(int)
    py = np.rint(height - 1 - (test[:, 1] - origin[1]) / resolution).astype(int)
    rows, cols = dist_map.shape
    valid = (px >= 0) & (px < cols) & (py >= 0) & (py < rows)
    if int(valid.sum()) < len(rotated) * 0.8:
        return None
    return float(dist_map[py[valid], px[valid]].mean())


def _best_translation(rotated, dist_map, origin, resolution, height, center, span):
    best_cost = None
    best_t = center
    for tx in np.arange(center[0] - span, center[0] + span + 1e-6, 0.25):
        for ty in np.arange(center[1] - span, center[1] + span + 1e-6, 0.25):
            translation = np.array([tx, ty])
            cost = _cost(
                rotated, translation, dist_map, origin, resolution, height
            )
            if cost is None:
                continue
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_t = translation
    return best_cost, best_t


def align() -> Alignment:
    with MAP_YAML.open() as handle:
        config = yaml.safe_load(handle)
    resolution = float(config['resolution'])
    origin = config['origin']
    image = cv2.imread(str(_image_path(config)), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(_image_path(config))
    height, width = image.shape

    wall_mask = (image < 50).astype(np.uint8)
    # Distance, in pixels, from every cell to the nearest occupied wall cell.
    dist_map = cv2.distanceTransform((1 - wall_mask) * 255, cv2.DIST_L2, 5)

    gazebo = _gazebo_points()
    spawn = gazebo.mean(axis=0) + np.array([0.0, LEFT_OFFSET_M])
    gazebo_yaw_deg = _rect_angle_deg(gazebo)

    wall_y, wall_x = np.nonzero(wall_mask)
    map_xy = np.column_stack((
        origin[0] + wall_x * resolution,
        origin[1] + (height - 1 - wall_y) * resolution,
    ))
    map_yaw_deg = _rect_angle_deg(map_xy)
    # A rectangle has the same walls at theta and theta+90. Try both.
    yaw_candidates = [
        math.radians(map_yaw_deg - gazebo_yaw_deg),
        math.radians(map_yaw_deg - gazebo_yaw_deg + 90.0),
    ]

    map_center = map_xy.mean(axis=0)
    notes = []
    best = None
    for yaw in yaw_candidates:
        rotated = _rotate(gazebo, yaw)
        centroid_t = map_center - rotated.mean(axis=0)
        cost, translation = _best_translation(
            rotated, dist_map, origin, resolution, height, centroid_t, span=4.0
        )
        notes.append(
            f'  yaw {math.degrees(yaw):7.2f} deg  '
            f'mean wall distance {cost:.2f} px  '
            f'T ({translation[0]:.2f}, {translation[1]:.2f})'
        )
        if cost is None:
            continue
        if best is None or cost < best[0]:
            best = (cost, yaw, translation, rotated)

    cost, yaw, translation, rotated = best
    # Fine yaw, and recompute translation at each step. The previous
    # translation is only the search center.
    for degree in np.linspace(math.degrees(yaw) - 2.0, math.degrees(yaw) + 2.0, 21):
        trial_yaw = math.radians(float(degree))
        trial_rot = _rotate(gazebo, trial_yaw)
        trial_cost, trial_t = _best_translation(
            trial_rot, dist_map, origin, resolution, height, translation, span=1.0
        )
        if trial_cost is not None and trial_cost < cost:
            cost, yaw, translation, rotated = trial_cost, trial_yaw, trial_t, trial_rot

    # OdometryPublisher sets odom -> base_link to the Gazebo world pose.
    # map -> odom is therefore the rigid fit itself: map = R * gazebo + T.
    cosine, sine = math.cos(yaw), math.sin(yaw)
    map_robot = np.array([
        cosine * spawn[0] - sine * spawn[1] + translation[0],
        sine * spawn[0] + cosine * spawn[1] + translation[1],
    ])

    max_x = origin[0] + width * resolution
    max_y = origin[1] + height * resolution
    report = '\n'.join([
        'align_map_to_gazebo.py',
        f'  map x range {origin[0]:.2f} .. {max_x:.2f}',
        f'  map y range {origin[1]:.2f} .. {max_y:.2f}',
        f'  gazebo wall long-edge angle {gazebo_yaw_deg:.2f} deg',
        f'  map wall long-edge angle {map_yaw_deg:.2f} deg',
        '  yaw candidates (map angle minus gazebo angle, and plus 90 deg):',
        *notes,
        f'  locked yaw {math.degrees(yaw):.2f} deg ({yaw:.4f} rad)',
        f'  rigid translation T ({translation[0]:.2f}, {translation[1]:.2f})',
        f'  mean distance of wall vertices to map walls: {cost:.2f} px',
        f'  spawn (wall center, {LEFT_OFFSET_M:.1f} m left) '
        f'({spawn[0]:.2f}, {spawn[1]:.2f}, 0.10)',
        f'  robot on map ({map_robot[0]:.2f}, {map_robot[1]:.2f})',
        f'  map -> odom T ({translation[0]:.2f}, {translation[1]:.2f}, 0.10) '
        f'yaw {yaw:.4f}',
    ])
    return Alignment(
        spawn_x=float(spawn[0]),
        spawn_y=float(spawn[1]),
        spawn_z=0.1,
        map_x=float(translation[0]),
        map_y=float(translation[1]),
        map_z=0.1,
        map_yaw=float(yaw),
        report=report,
    )


def main():
    print(align().report)


if __name__ == '__main__':
    main()
