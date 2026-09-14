from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest


ARM_JOINTS = {f'joint{index}' for index in range(1, 7)}


@pytest.mark.parametrize('mock', ['true', 'false'])
def test_mit_interfaces(mock):
    result = subprocess.run([
        'xacro', str(Path(__file__).parents[1] / 'urdf/rebot_arm_rs.urdf.xacro'),
        f'use_mock_hardware:={mock}',
    ], check=True, capture_output=True, text=True)
    control = ET.fromstring(result.stdout).find('ros2_control')
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
    xacro_file = (
        Path(__file__).parents[1] / 'urdf' / 'rebot_arm_rs.urdf.xacro'
    )
    result = subprocess.run(
        [
            'xacro',
            str(xacro_file),
            'use_isaac_sim:=true',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = ET.fromstring(result.stdout)
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
