"""Run real rclpy service calls in an isolated ROS domain with fake controllers.
No SocketCAN socket is opened and no real controller manager is contacted.
"""
import os
import time
from collections import deque
import pytest

# Explicitly refuse the user's normal ROS domain, including accidental colcon tests.
if os.environ.get('ROS_DOMAIN_ID') != '187':
    pytest.skip('integration test requires isolated ROS_DOMAIN_ID=187', allow_module_level=True)
rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_action_status_default
from controller_manager_msgs.srv import ListControllers, SwitchController
from controller_manager_msgs.msg import ControllerState
from action_msgs.msg import GoalStatusArray, GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
from rebot_arm_hmi_bridge.node import HmiBridge
from rebot_arm_hmi_bridge.core import BASE, KNOWN, CHAINS, JTC
from rebot_arm_hmi_bridge.protocol import STATE, RESULT, READY, Mode, Result, packet


class MemoryCan:
    def __init__(self):
        self.input, self.output = deque(), []
    def receive(self):
        return self.input.popleft() if self.input else None
    def send(self, can_id, data):
        self.output.append((can_id, data))
    def close(self):
        pass


class Manager(Node):
    def __init__(self):
        super().__init__('isolated_hmi_test_manager')
        self.current = Mode.POSITION
        self.calls = []
        self.reject = self.lie = self.trajectory = False
        self.create_service(ListControllers, '/controller_manager/list_controllers', self.list)
        self.create_service(SwitchController, '/controller_manager/switch_controller', self.switch)
        self.state_pubs = [self.create_publisher(JointTrajectoryControllerState, '/' + n + '/controller_state', 10) for n in JTC]
        self.status_pubs = [self.create_publisher(GoalStatusArray, '/' + n + '/follow_joint_trajectory/_action/status', qos_profile_action_status_default) for n in JTC]
        self.create_timer(.02, self.publish)
    def list(self, req, response):
        response.controller = [ControllerState(name=n, state='active' if n == BASE or n in CHAINS[self.current] else 'inactive') for n in [BASE, *KNOWN]]
        return response
    def switch(self, req, response):
        self.calls.append(req)
        assert req.strictness == 2 and not req.activate_asap and req.timeout.sec == 2
        assert BASE not in req.activate_controllers + req.deactivate_controllers
        response.ok = not self.reject
        if response.ok and not self.lie:
            self.current = next(m for m, chain in CHAINS.items() if set(chain) == set(req.activate_controllers))
        return response
    def publish(self):
        msg = JointTrajectoryControllerState()
        msg.reference.velocities = msg.feedback.velocities = [0.] * 6
        status = GoalStatusArray()
        if self.trajectory:
            goal = GoalStatus(status=2)
            goal.goal_info.goal_id.uuid = [1] * 16
            status.status_list = [goal]
        for pub in self.state_pubs:
            pub.publish(msg)
        for pub in self.status_pubs:
            pub.publish(status)


@pytest.fixture
def rig():
    rclpy.init()
    manager, transport = Manager(), MemoryCan()
    bridge = HmiBridge(transport)
    executor = SingleThreadedExecutor()
    executor.add_node(manager); executor.add_node(bridge)
    def spin(condition, seconds=4):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if condition():
                return
        raise AssertionError(f'timed out; phase={bridge.phase} outputs={transport.output[-5:]}')
    spin(lambda: bridge.ready(time.monotonic())[0])
    yield manager, bridge, transport, spin
    executor.shutdown(); bridge.destroy_node(); manager.destroy_node(); rclpy.shutdown()


def send(bridge, transport, mode):
    token = bridge.leases.token
    transport.input.append(packet(mode, token))
    return token


def final(transport, token):
    return [data for can_id, data in transport.output
            if can_id == RESULT and int.from_bytes(data[2:6], 'little') == token and data[6] != Result.ACCEPTED]


def test_verified_switch_and_duplicate(rig):
    manager, bridge, transport, spin = rig
    token = send(bridge, transport, Mode.DAMPING)
    spin(lambda: bool(final(transport, token)))
    assert final(transport, token)[-1][6:] == bytes([Result.DONE, Mode.DAMPING])
    assert len(manager.calls) == 1
    transport.input.append(packet(Mode.DAMPING, token))
    spin(lambda: len(final(transport, token)) == 2)
    assert len(manager.calls) == 1
    token = send(bridge, transport, Mode.IMPEDANCE)
    spin(lambda: bool(final(transport, token)))
    assert final(transport, token)[-1][6:] == bytes([Result.DONE, Mode.IMPEDANCE])
    assert len(manager.calls) == 2


def test_trajectory_refused_without_cancel_or_switch(rig):
    manager, bridge, transport, spin = rig
    manager.trajectory = True
    spin(lambda: any(bridge.status.values()))
    token = send(bridge, transport, Mode.DAMPING)
    spin(lambda: bool(final(transport, token)))
    assert final(transport, token)[-1][6] == Result.TRAJECTORY_BUSY
    assert not manager.calls and manager.trajectory


@pytest.mark.parametrize('reject,expected', [(True, Result.SWITCH_FAILED), (False, Result.VERIFY_FAILED)])
def test_failure_is_not_success(rig, reject, expected):
    manager, bridge, transport, spin = rig
    manager.reject, manager.lie = reject, not reject
    token = send(bridge, transport, Mode.DAMPING)
    spin(lambda: bool(final(transport, token)))
    assert final(transport, token)[-1][6] == expected
    assert manager.current == Mode.POSITION and len(manager.calls) == 1


def test_direct_topic_writers_block_switch(rig):
    manager, bridge, transport, spin = rig
    publisher = manager.create_publisher(JointTrajectory, '/' + JTC[0] + '/joint_trajectory', 10)
    spin(lambda: not bridge.trajectory_guard(time.monotonic())[1])
    token = send(bridge, transport, Mode.DAMPING)
    spin(lambda: bool(final(transport, token)))
    assert final(transport, token)[-1][6] == Result.NOT_READY
    assert not manager.calls
    manager.destroy_publisher(publisher)


def test_timeout_quarantines_unresolved_service(rig):
    manager, bridge, transport, spin = rig
    from rclpy.task import Future
    # Inject an unresolved service future without sending any ROS switch request.
    token = bridge.leases.token
    bridge.leases.begin(2, token, time.monotonic(), bridge.mode)
    pending = Future()
    bridge.future, bridge.phase, bridge.deadline = pending, 'switch', time.monotonic()-.1
    spin(lambda: bool(final(transport, token)))
    assert final(transport, token)[-1][6] == Result.SERVICE_TIMEOUT
    assert bridge.phase == 'quarantine'
    another = send(bridge, transport, Mode.IMPEDANCE)
    spin(lambda: bool(final(transport, another)))
    assert final(transport, another)[-1][6] == Result.REQUEST_BUSY
    assert not manager.calls
    pending.set_result(SwitchController.Response(ok=True))
    spin(lambda: bridge.valid_config and bridge.phase != 'quarantine')
    assert not manager.calls


def test_already_selected_and_invalid_requests_do_not_switch(rig):
    manager, bridge, transport, spin = rig
    token = send(bridge, transport, Mode.POSITION)
    spin(lambda: bool(final(transport, token)))
    assert final(transport, token)[-1][6] == Result.DONE and not manager.calls
    transport.input.append(packet(Mode.DAMPING, token + 100))
    spin(lambda: bool(final(transport, token + 100)))
    assert final(transport, token + 100)[-1][6] == Result.STALE_TOKEN and not manager.calls
    lease = bridge.leases.token
    transport.input.append(packet(4, lease))
    spin(lambda: not transport.input)
    assert bridge.leases.token == lease and not manager.calls


def test_goal_arriving_during_preflight_is_rejected(rig):
    manager, bridge, transport, spin = rig
    token = send(bridge, transport, Mode.DAMPING)
    spin(lambda: bridge.phase == 'preflight')
    manager.trajectory = True
    msg = GoalStatusArray()
    msg.status_list = [GoalStatus(status=1)]
    bridge.goal_status(JTC[0], msg)
    spin(lambda: bool(final(transport, token)))
    assert final(transport, token)[-1][6] == Result.TRAJECTORY_BUSY
    assert not manager.calls
