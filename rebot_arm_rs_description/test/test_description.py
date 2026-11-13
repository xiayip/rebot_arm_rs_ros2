from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest


ARM_JOINTS = {f'joint{index}' for index in range(1, 7)}
DESCRIPTION_ROOT = Path(__file__).parents[1]


def expand_description(filename, *arguments):
    result = subprocess.run(
        ['xacro', str(DESCRIPTION_ROOT / 'urdf' / filename), *arguments],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


@pytest.mark.parametrize('mock', ['true', 'false'])
def test_mit_interfaces(mock):
    robot = expand_description(
        'rebot_arm_rs.urdf.xacro', f'use_mock_hardware:={mock}'
    )
    control = robot.find('ros2_control')
    assert control.findtext('hardware/plugin') == (
        'mock_components/GenericSystem' if mock == 'true' else 'shared_can_ros2/CanSystem')
    joints = control.findall('joint')
    assert {j.attrib['name'] for j in joints} == ARM_JOINTS
    for joint in joints:
        interfaces = {i.attrib['name'] for i in joint.findall('command_interface')}
        assert interfaces == {'position', 'velocity', 'kp', 'kd', 't_ff'}
        params = {p.attrib['name']: p.text for p in joint.findall('param')}
        assert params['protocol_plugin'] == 'rebot_robstride_protocol/RobStrideProtocol'
        assert joint.find("command_interface[@name='position']/param[@name='initial_value']") is None


def test_isaac_description_uses_standard_topic_based_interfaces():
    robot = expand_description(
        'rebot_arm_rs.urdf.xacro', 'use_isaac_sim:=true'
    )
    control = robot.find('ros2_control')
    assert control is not None
    assert control.findtext('hardware/plugin') == (
        'topic_based_ros2_control/TopicBasedSystem'
    )

    parameters = {
        parameter.attrib['name']: parameter.text
        for parameter in control.findall('hardware/param')
    }
    assert parameters['joint_commands_topic'] == (
        '/rebot_arm_rs/isaac_joint_commands'
    )
    assert parameters['joint_states_topic'] == (
        '/rebot_arm_rs/isaac_joint_states'
    )
    assert parameters['publish_full_joint_command_arrays'] == 'true'

    joints = {joint.attrib['name']: joint for joint in control.findall('joint')}
    assert joints.keys() == ARM_JOINTS
    for joint in joints.values():
        assert [item.attrib['name'] for item in joint.findall('command_interface')] == [
            'position'
        ]
        assert {
            item.attrib['name'] for item in joint.findall('state_interface')
        } == {'position', 'velocity', 'effort'}


def test_official_parallel_gripper_matches_upstream_model():
    robot = expand_description(
        'rebot_arm_rs_with_official_parallel_gripper.urdf.xacro',
        'use_mock_hardware:=true',
    )
    links = {link.attrib['name']: link for link in robot.findall('link')}
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}

    assert {'gripper_end', 'gripper_left', 'gripper_right'} <= links.keys()
    assert 'gripper_control_link' in links

    mount = joints['j_gripper_end']
    assert mount.attrib['type'] == 'fixed'
    assert mount.find('parent').attrib['link'] == 'rebot_arm_rs_stock_flange_link'
    assert mount.find('child').attrib['link'] == 'gripper_end'
    assert [float(value) for value in mount.find('origin').attrib['xyz'].split()] == [
        0.0, 0.0, 0.16621
    ]
    assert [float(value) for value in mount.find('origin').attrib['rpy'].split()] == [
        3.1416, -1.5708, 0.0
    ]

    expected_fingers = {
        'gripper_joint1': (
            'gripper_left',
            [-0.041939, -7.3385e-05, 0.0],
            [1.5708, -1.5708, 0.0],
            0.05,
        ),
        'gripper_joint2': (
            'gripper_right',
            [-0.041939, 7.3385e-05, 0.0],
            [-1.5708, -1.5708, 0.0],
            0.0715,
        ),
    }
    for joint_name, (child, xyz, rpy, upper) in expected_fingers.items():
        joint = joints[joint_name]
        assert joint.attrib['type'] == 'prismatic'
        assert joint.find('parent').attrib['link'] == 'gripper_end'
        assert joint.find('child').attrib['link'] == child
        assert [float(value) for value in joint.find('origin').attrib['xyz'].split()] == xyz
        assert [float(value) for value in joint.find('origin').attrib['rpy'].split()] == rpy
        assert joint.find('axis').attrib['xyz'] == '0 0 1'
        assert float(joint.find('limit').attrib['lower']) == 0.0
        assert float(joint.find('limit').attrib['upper']) == upper
        assert float(joint.find('limit').attrib['effort']) == 500.0
        assert float(joint.find('limit').attrib['velocity']) == 10.0
        mimic = joint.find('mimic')
        assert mimic.attrib['joint'] == 'gripper_command_joint'
        assert float(mimic.attrib['multiplier']) == -upper
        assert float(mimic.attrib['offset']) == upper

    command_joint = joints['gripper_command_joint']
    assert command_joint.attrib['type'] == 'revolute'
    assert command_joint.find('parent').attrib['link'] == 'gripper_end'
    assert command_joint.find('child').attrib['link'] == 'gripper_control_link'
    assert float(command_joint.find('limit').attrib['lower']) == 0.0
    assert float(command_joint.find('limit').attrib['upper']) == 1.0

    assert float(links['gripper_end'].find('inertial/mass').attrib['value']) == 0.65
    assert float(links['gripper_left'].find('inertial/mass').attrib['value']) == 0.0752
    assert float(links['gripper_right'].find('inertial/mass').attrib['value']) == 0.0752

    control_joints = {
        joint.attrib['name'] for joint in robot.find('ros2_control').findall('joint')
    }
    assert control_joints == ARM_JOINTS | {'gripper_command_joint'}
    control_gripper = robot.find(
        "ros2_control/joint[@name='gripper_command_joint']"
    )
    assert {
        interface.attrib['name']
        for interface in control_gripper.findall('command_interface')
    } == {'position', 'velocity'}
    parameters = {
        parameter.attrib['name']: parameter.text
        for parameter in control_gripper.findall('param')
    }
    assert parameters['protocol_plugin'] == (
        'rebot_robstride_protocol/RebotGripperProtocol'
    )
    assert parameters['motor_id'] == '0x07'
    assert float(parameters['open_position']) == 5.0
    assert float(parameters['closed_position']) == 0.0


def test_official_parallel_gripper_meshes_are_packaged():
    robot = expand_description(
        'rebot_arm_rs_with_official_parallel_gripper.urdf.xacro',
        'use_mock_hardware:=true',
    )
    package_prefix = (
        'package://rebot_arm_rs_description/meshes/rebot_arm_rs/'
        'official_parallel_gripper/'
    )
    mesh_names = {
        mesh.attrib['filename'].removeprefix(package_prefix)
        for mesh in robot.findall('.//mesh')
        if mesh.attrib['filename'].startswith(package_prefix)
    }
    assert mesh_names == {
        'cnc7.STL',
        'cnc_left.STL',
        'cnc_right.STL',
        'gripper_end.STL',
        'gripper_left.STL',
        'gripper_right.STL',
        'motor_7.STL',
        'pla7_green.STL',
        'pla_left.STL',
        'pla_right.STL',
    }
    mesh_directory = (
        DESCRIPTION_ROOT / 'meshes' / 'rebot_arm_rs' / 'official_parallel_gripper'
    )
    assert all((mesh_directory / mesh_name).is_file() for mesh_name in mesh_names)


def test_default_description_remains_without_official_gripper():
    robot = expand_description('rebot_arm_rs.urdf.xacro', 'use_mock_hardware:=true')
    link_names = {link.attrib['name'] for link in robot.findall('link')}
    joint_names = {joint.attrib['name'] for joint in robot.findall('joint')}
    assert 'gripper_end' not in link_names
    assert 'gripper_joint1' not in joint_names
    assert 'gripper_joint2' not in joint_names


@pytest.mark.parametrize('name', ['base_link', 'link1', 'link2', 'link3', 'link4', 'link5'])
def test_collision_mesh_preserves_original_surface(name):
    import struct
    import numpy as np

    mesh_dir = DESCRIPTION_ROOT / 'meshes/rebot_arm_rs'

    def read_stl(path):
        data = path.read_bytes()
        count = struct.unpack('<I', data[80:84])[0]
        assert len(data) == 84 + 50 * count
        return np.frombuffer(data, offset=84, count=count, dtype=np.dtype([
            ('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('part', '<u2')]))

    original = read_stl(mesh_dir / (name + '.STL'))
    reduced = read_stl(mesh_dir / 'collision' / (name + '.STL'))
    assert 0 < len(reduced) < 6000
    assert len(reduced) < len(original) / 5
    remaining = original['vertices'].astype(float)
    # Each disconnected convex shell carries a piece ID in the STL attribute.
    # Every original triangle must fit entirely inside at least one shell.
    for part_id in np.unique(reduced['part']):
        assert part_id > 0
        part = reduced[reduced['part'] == part_id]
        triangles = part['vertices'].astype(float)
        normals = np.cross(triangles[:, 1] - triangles[:, 0],
                           triangles[:, 2] - triangles[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        assert np.all(lengths > 1e-18)
        normals /= lengths[:, None]
        assert np.all(np.sum(normals * part['normal'], axis=1) > .99)
        offsets = np.sum(normals * triangles[:, 0], axis=1)
        assert np.max(triangles.reshape(-1, 3) @ normals.T - offsets) < 1e-7
        kept = []
        for chunk in np.array_split(remaining, max(1, len(remaining) // 2048)):
            if not len(chunk):
                continue
            distances = (chunk.reshape(-1, 3) @ normals.T - offsets).reshape(
                len(chunk), 3, -1).max(axis=(1, 2))
            kept.append(chunk[distances > -0.00005])
        remaining = np.concatenate(kept) if kept else np.empty((0, 3, 3))
    assert len(remaining) == 0


def test_arm_uses_lightweight_collisions_and_original_visuals():
    robot = expand_description('rebot_arm_rs.urdf.xacro', 'use_mock_hardware:=true')
    prefix = 'package://rebot_arm_rs_description/meshes/rebot_arm_rs/'
    for name in ['base_link', 'link1', 'link2', 'link3', 'link4', 'link5']:
        collision = robot.find(f"link[@name='{name}']/collision/geometry/mesh")
        assert collision.get('filename') == prefix + 'collision/' + name + '.STL'
    visuals = robot.findall('.//visual/geometry/mesh')
    assert visuals
    assert all('/collision/' not in mesh.get('filename') for mesh in visuals)
