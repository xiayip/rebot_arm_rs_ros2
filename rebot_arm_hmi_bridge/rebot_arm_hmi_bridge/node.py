"""Standalone bridge. No motor commands, goal cancellation or implicit rollback."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers, SwitchController
from action_msgs.msg import GoalStatusArray
from control_msgs.msg import JointTrajectoryControllerState

from .core import CHAINS, JTC, Leases, inspect, plan
from .protocol import STATE, RESULT, READY, BUSY, TRAJECTORY, FAULT, Mode, Result, packet, request
from .transport import CanTransport


class HmiBridge(Node):
    def __init__(self, transport=None):
        super().__init__('rebot_arm_hmi_bridge')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('controller_namespace', '/')
        manager = self.get_parameter('controller_manager').value.rstrip('/')
        namespace = self.get_parameter('controller_namespace').value.rstrip('/')
        self.transport = transport or CanTransport(self.get_parameter('can_interface').value)
        self.list_client = self.create_client(ListControllers, manager + '/list_controllers')
        self.switch_client = self.create_client(SwitchController, manager + '/switch_controller')
        self.names = {name: namespace + '/' + name for name in JTC}
        self.status = {name: set() for name in JTC}
        self.telemetry = {name: (0., False) for name in JTC}
        self.matched_since = {name: None for name in JTC}
        self.quiet_since = {name: None for name in JTC}
        self.subscriptions_owned = []
        for name in JTC:
            self.subscriptions_owned.append(self.create_subscription(
                GoalStatusArray, self.names[name] + '/follow_joint_trajectory/_action/status',
                lambda msg, n=name: self.goal_status(n, msg), qos_profile_action_status_default))
            self.subscriptions_owned.append(self.create_subscription(
                JointTrajectoryControllerState, self.names[name] + '/controller_state',
                lambda msg, n=name: self.controller_state(n, msg), qos_profile_sensor_data))
        now = time.monotonic()
        self.leases = Leases(now)
        self.controllers = []
        self.mode, self.capabilities = Mode.UNKNOWN, 0
        self.list_at = 0.
        self.valid_config = False
        self.future = None
        self.phase = 'idle'
        self.deadline = 0.
        self.next_list, self.next_heartbeat = now, now
        self.last_transport_warning = 0.
        self.timer = self.create_timer(.02, self.tick)
        self.get_logger().info('HMI bridge ready to observe; controller switching requires an explicit CAN request')

    def goal_status(self, name, msg):
        self.status[name] = {bytes(s.goal_info.goal_id.uuid) for s in msg.status_list
                             if s.status in (0, 1, 2, 3)}

    def controller_state(self, name, msg):
        # Fresh stationary reference AND feedback are required before switching.
        values = list(msg.reference.velocities) + list(msg.feedback.velocities)
        stationary = (len(msg.reference.velocities) == 6 and len(msg.feedback.velocities) == 6
                      and all(math.isfinite(v) and abs(v) < .01 for v in values))
        self.telemetry[name] = time.monotonic(), stationary

    def trajectory_guard(self, now):
        """(in-flight, observable). Direct topic writers lack a queryable goal state."""
        moving = any(self.status.values())
        observable = True
        for name, prefix in self.names.items():
            # Conservatively refuse while direct topic command publishers exist,
            # including future-stamped trajectories a late subscriber cannot see.
            if self.count_publishers(prefix + '/joint_trajectory'):
                observable = False
            active = any(c.name == name and c.state == 'active' for c in self.controllers)
            if not active:
                self.quiet_since[name] = self.matched_since[name] = None
                continue
            matched = self.count_publishers(prefix + '/follow_joint_trajectory/_action/status') > 0
            if not matched:
                self.matched_since[name] = None
            elif self.matched_since[name] is None:
                self.matched_since[name] = now
            at, stationary = self.telemetry[name]
            fresh = now-at < .5
            if not fresh or not stationary or self.status[name]:
                self.quiet_since[name] = None
            elif self.quiet_since[name] is None:
                self.quiet_since[name] = now
            if not fresh:
                observable = False
            elif not stationary:
                moving = True
            if (self.matched_since[name] is None or now-self.matched_since[name] < 1.0
                    or self.quiet_since[name] is None or now-self.quiet_since[name] < .5):
                observable = False
        return moving, observable

    def ready(self, now):
        moving, observable = self.trajectory_guard(now)
        return (self.valid_config and now-self.list_at < .75 and observable
                and not moving and self.switch_client.service_is_ready()), moving

    def send(self, can_id, data):
        try:
            self.transport.send(can_id, data)
        except OSError as exc:
            if time.monotonic()-self.last_transport_warning > 2:
                self.get_logger().warning(f'CAN output failed: {exc}')
                self.last_transport_warning = time.monotonic()

    def finish(self, result, now):
        if self.leases.current is not None:
            target, _ = self.leases.current
            self.get_logger().info(f'HMI target={target} result={Result(result).name} observed={int(self.mode)}')
            self.send(RESULT, self.leases.finish(result, self.mode, now))

    def start_list(self, phase, now):
        if not self.list_client.service_is_ready():
            self.valid_config = False
            self.finish(Result.NOT_READY, now)
            self.phase = 'idle'
            self.next_list = now + .2
            return
        self.future = self.list_client.call_async(ListControllers.Request())
        self.phase, self.deadline = phase, now + .6

    def update_controllers(self, response, now):
        self.controllers = response.controller
        self.list_at = now
        try:
            self.mode, self.capabilities = inspect(self.controllers)
            self.valid_config = True
        except ValueError:
            self.mode, self.capabilities, self.valid_config = Mode.UNKNOWN, 0, False

    def progress(self, now):
        if self.future is None:
            return
        if not self.future.done():
            if now >= self.deadline and self.phase != 'quarantine':
                self.valid_config = False
                self.finish(Result.SERVICE_TIMEOUT, now)
                # A timed-out service may still mutate the controller manager.
                # Retain its future and deny new requests until it resolves.
                self.phase = 'quarantine'
            return
        future, phase = self.future, self.phase
        self.future = None
        self.phase = 'idle'
        try:
            response = future.result()
            if phase == 'quarantine':
                self.start_list('poll', now)
            elif phase == 'switch':
                if not response.ok:
                    self.valid_config = False
                    self.finish(Result.SWITCH_FAILED, now)
                    self.start_list('poll', now)
                else:
                    self.start_list('verify', now)
            else:
                self.update_controllers(response, now)
                self.next_list = now + .2
                if phase == 'verify':
                    target, _ = self.leases.current
                    self.finish(Result.DONE if self.valid_config and self.mode == target
                                else Result.VERIFY_FAILED, now)
                elif phase == 'preflight':
                    target, _ = self.leases.current
                    ready, moving = self.ready(now)
                    if moving:
                        self.finish(Result.TRAJECTORY_BUSY, now)
                    elif not self.valid_config:
                        self.finish(Result.UNKNOWN_CONFIG, now)
                    elif not ready:
                        self.finish(Result.NOT_READY, now)
                    else:
                        activate, deactivate = plan(self.controllers, target)
                        if not activate:
                            self.finish(Result.DONE, now)
                            return
                        req = SwitchController.Request()
                        req.activate_controllers, req.deactivate_controllers = activate, deactivate
                        req.strictness = 2
                        req.activate_asap = False
                        req.timeout.sec = 2
                        self.future = self.switch_client.call_async(req)
                        self.phase, self.deadline = 'switch', now + 2.5
        except Exception as exc:
            self.get_logger().error(f'Controller service failed: {exc}')
            self.valid_config = False
            self.finish(Result.VERIFY_FAILED if phase == 'verify' else Result.SWITCH_FAILED, now)

    def incoming(self, data, now):
        parsed = request(data)
        if parsed is None:
            return  # Invalid frames never consume a lease or call ROS services.
        target, token = parsed
        first, reply = self.leases.begin(target, token, now, self.mode)
        if not first:
            self.send(RESULT, reply)
            return
        self.send(RESULT, reply)
        ready, moving = self.ready(now)
        if self.phase not in ('idle', 'poll'):
            self.finish(Result.REQUEST_BUSY, now)
        elif moving:
            self.finish(Result.TRAJECTORY_BUSY, now)
        elif not ready:
            self.finish(Result.NOT_READY, now)
        else:
            # An already in-flight list response serves as fresh preflight.
            if self.phase == 'poll':
                self.phase = 'preflight'
            else:
                self.start_list('preflight', now)

    def tick(self):
        now = time.monotonic()
        self.leases.expire(now)
        self.progress(now)
        for _ in range(16):
            try:
                data = self.transport.receive()
            except OSError:
                break
            if data is None:
                break
            self.incoming(data, now)
        if self.phase == 'idle' and now >= self.next_list:
            self.start_list('poll', now)
        if now >= self.next_heartbeat:
            ready, moving = self.ready(now)
            busy = self.leases.current is not None or self.phase == 'quarantine'
            flags = ((READY if ready and not busy else 0) | (BUSY if busy else 0)
                     | (TRAJECTORY if moving else 0)
                     | (FAULT if not self.valid_config or now-self.list_at >= .75 else 0))
            mode = self.mode if now-self.list_at < .75 else Mode.UNKNOWN
            self.send(STATE, packet(mode, self.leases.token, flags, self.capabilities))
            self.next_heartbeat = now + .2

    def destroy_node(self):
        self.transport.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HmiBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
