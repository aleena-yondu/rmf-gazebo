#!/usr/bin/env python3
"""Print a visual-only URDF for an SDF model, for robot_state_publisher and RViz.

    python3 scripts/sdf_to_urdf.py models/robot_1/model.sdf

Only links, fixed or moving joints, and mesh/box/cylinder/sphere visuals are
converted. model:// URIs resolve against the directory that holds the model.
"""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def _matrix(pose_text):
    values = [float(v) for v in (pose_text or '0 0 0 0 0 0').split()]
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = [x, y, z]
    return transform


def _xyz_rpy(transform):
    rotation = transform[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    xyz = ' '.join(f'{v:.6g}' for v in transform[:3, 3])
    return xyz, f'{roll:.6g} {pitch:.6g} {yaw:.6g}'


def convert(sdf_path):
    sdf_path = Path(sdf_path).resolve()
    models_dir = sdf_path.parent.parent
    model = ET.parse(sdf_path).getroot().find('model')
    links = {link.get('name'): link for link in model.findall('link')}

    model_frame = {}

    def pose_in_model(name):
        if name in model_frame:
            return model_frame[name]
        pose = links[name].find('pose')
        local = _matrix(pose.text if pose is not None else None)
        parent = pose.get('relative_to') if pose is not None else None
        model_frame[name] = (pose_in_model(parent) if parent else np.eye(4)) @ local
        return model_frame[name]

    robot = ET.Element('robot', name=model.get('name'))
    for name, link in links.items():
        urdf_link = ET.SubElement(robot, 'link', name=name)
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            urdf_geometry = ET.Element('geometry')
            mesh = geometry.find('mesh')
            if mesh is not None:
                uri = mesh.find('uri').text.strip()
                if uri.startswith('model://'):
                    uri = 'file://' + str(models_dir / uri[len('model://'):])
                attributes = {'filename': uri}
                scale = mesh.find('scale')
                if scale is not None:
                    attributes['scale'] = scale.text.strip()
                ET.SubElement(urdf_geometry, 'mesh', attributes)
            elif geometry.find('box') is not None:
                ET.SubElement(urdf_geometry, 'box',
                              size=geometry.find('box/size').text.strip())
            elif geometry.find('cylinder') is not None:
                ET.SubElement(urdf_geometry, 'cylinder',
                              radius=geometry.find('cylinder/radius').text.strip(),
                              length=geometry.find('cylinder/length').text.strip())
            elif geometry.find('sphere') is not None:
                ET.SubElement(urdf_geometry, 'sphere',
                              radius=geometry.find('sphere/radius').text.strip())
            else:
                continue
            urdf_visual = ET.SubElement(urdf_link, 'visual')
            pose = visual.find('pose')
            xyz, rpy = _xyz_rpy(_matrix(pose.text if pose is not None else None))
            ET.SubElement(urdf_visual, 'origin', xyz=xyz, rpy=rpy)
            urdf_visual.append(urdf_geometry)

    for joint in model.findall('joint'):
        parent = joint.find('parent').text.strip()
        child = joint.find('child').text.strip()
        offset = np.linalg.inv(pose_in_model(parent)) @ pose_in_model(child)
        joint_type = joint.get('type')
        if joint_type == 'revolute' and joint.find('axis/limit') is None:
            joint_type = 'continuous'
        urdf_joint = ET.SubElement(robot, 'joint', name=joint.get('name'), type=joint_type)
        ET.SubElement(urdf_joint, 'parent', link=parent)
        ET.SubElement(urdf_joint, 'child', link=child)
        xyz, rpy = _xyz_rpy(offset)
        ET.SubElement(urdf_joint, 'origin', xyz=xyz, rpy=rpy)
        axis = joint.find('axis/xyz')
        if joint_type != 'fixed' and axis is not None:
            ET.SubElement(urdf_joint, 'axis', xyz=axis.text.strip())
            limit = joint.find('axis/limit')
            if limit is not None and joint_type in ('revolute', 'prismatic'):
                ET.SubElement(urdf_joint, 'limit',
                              lower=limit.findtext('lower', '0'),
                              upper=limit.findtext('upper', '0'),
                              effort=limit.findtext('effort', '0'),
                              velocity=limit.findtext('velocity', '0'))

    ET.indent(robot)
    return ET.tostring(robot, encoding='unicode')


if __name__ == '__main__':
    print(convert(sys.argv[1]))
