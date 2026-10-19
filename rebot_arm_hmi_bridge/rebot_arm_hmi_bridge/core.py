"""No ROS or CAN side effects: controller planning and one-use request leases."""
import secrets
from collections import OrderedDict
from .protocol import Mode, Result, packet

BASE = 'rebot_arm_rs_mit_controller'
CHAINS = {
    Mode.POSITION: ('rebot_arm_rs_position_adapter', 'rebot_arm_rs_trajectory_controller'),
    Mode.DAMPING: ('rebot_arm_rs_damping_controller',),
    Mode.IMPEDANCE: ('rebot_arm_rs_impedance_controller', 'rebot_arm_rs_impedance_trajectory_controller'),
}
JTC = (CHAINS[Mode.POSITION][-1], CHAINS[Mode.IMPEDANCE][-1])
KNOWN = {name for chain in CHAINS.values() for name in chain}


def inspect(controllers):
    """Return observed mode and available modes; reject mixed/unknown owners."""
    states = {c.name: c.state for c in controllers}
    if states.get(BASE) != 'active':
        raise ValueError('base MIT controller is not active')
    for c in controllers:
        if c.state != 'active' or c.name in KNOWN or c.name == BASE:
            continue
        for interface in c.claimed_interfaces:
            owner = interface.split('/')[0]
            if owner in {BASE, *KNOWN, *(f'joint{j}' for j in range(1, 7))}:
                raise ValueError(f'unknown arm owner: {c.name}')
    active = {name for name in KNOWN if states.get(name) == 'active'}
    modes = [mode for mode, names in CHAINS.items() if active == set(names)]
    if len(modes) != 1:
        raise ValueError('no single complete active arm chain')
    capabilities = sum(1 << (mode-1) for mode, names in CHAINS.items()
                       if all(states.get(name) in ('active', 'inactive') for name in names))
    return modes[0], capabilities


def plan(controllers, target):
    mode, capabilities = inspect(controllers)
    if target not in CHAINS or not capabilities & (1 << (target-1)):
        raise ValueError('target chain is not configured')
    if mode == target:
        return [], []
    # Downstream-to-upstream activation; stop upstream before its adapter.
    return list(CHAINS[target]), list(reversed(CHAINS[mode]))


class Leases:
    """A duplicate request returns its stored reply and never repeats a switch."""
    def __init__(self, now, token=None):
        self.token = token or secrets.randbelow(0xFFFFFFFF) + 1
        self.expires = now + 5.0
        self.cache = OrderedDict()
        self.current = None

    def rotate(self, now):
        self.token = self.token % 0xFFFFFFFF + 1
        self.expires = now + 5.0

    def expire(self, now):
        if self.current is None and now >= self.expires:
            self.rotate(now)
        for key, (_, _, at) in list(self.cache.items()):
            if now-at > 60 and (self.current is None or key != self.current[1]):
                del self.cache[key]

    def begin(self, mode, token, now, observed):
        self.expire(now)
        if token in self.cache:
            old_mode, reply, _ = self.cache[token]
            return False, reply if mode == old_mode else packet(mode, token, Result.BAD_REQUEST, observed)
        if self.current is not None:
            return False, packet(mode, token, Result.REQUEST_BUSY, observed)
        if token != self.token:
            return False, packet(mode, token, Result.STALE_TOKEN, observed)
        self.current = mode, token
        reply = packet(mode, token, Result.ACCEPTED, observed)
        self.cache[token] = mode, reply, now
        while len(self.cache) > 64:
            self.cache.popitem(last=False)
        return True, reply

    def finish(self, result, observed, now):
        mode, token = self.current
        reply = packet(mode, token, result, observed)
        self.cache[token] = mode, reply, now
        self.current = None
        self.rotate(now)
        return reply
