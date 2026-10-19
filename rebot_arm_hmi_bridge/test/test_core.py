from types import SimpleNamespace as C
from rebot_arm_hmi_bridge.core import BASE, CHAINS, KNOWN, Leases, inspect, plan
from rebot_arm_hmi_bridge.protocol import Mode, Result, packet, request
import pytest


def controllers(mode):
    return [C(name=n, state='active' if n == BASE or n in CHAINS[mode] else 'inactive',
              claimed_interfaces=[]) for n in [BASE, *KNOWN]]


def test_all_transitions():
    for start in CHAINS:
        for target in CHAINS:
            observed, available = inspect(controllers(start))
            assert observed == start and available == 7
            activate, deactivate = plan(controllers(start), target)
            assert BASE not in activate + deactivate
            if target == start:
                assert not activate and not deactivate
            else:
                assert activate == list(CHAINS[target])
                assert deactivate == list(reversed(CHAINS[start]))


def test_reject_unknown_partial_and_unconfigured():
    items = controllers(Mode.POSITION)
    items.append(C(name='unknown', state='active', claimed_interfaces=['joint2/position']))
    with pytest.raises(ValueError):
        inspect(items)
    items = controllers(Mode.POSITION)
    items[0].state = 'inactive'
    with pytest.raises(ValueError):
        inspect(items)
    items = controllers(Mode.POSITION)
    next(c for c in items if c.name == CHAINS[Mode.DAMPING][0]).state = 'active'
    with pytest.raises(ValueError):
        inspect(items)
    items = controllers(Mode.DAMPING)
    next(c for c in items if c.name == CHAINS[Mode.POSITION][0]).state = 'unconfigured'
    assert inspect(items)[1] == 6
    with pytest.raises(ValueError):
        plan(items, Mode.POSITION)


def test_wire_golden_and_malformed():
    golden = bytes.fromhex('0102123456780000')
    assert packet(2, 0x78563412) == golden
    assert request(golden) == (2, 0x78563412)
    for index, value in [(0, 2), (1, 0), (1, 4), (6, 1), (7, 1)]:
        bad = bytearray(golden); bad[index] = value
        assert request(bad) is None
    assert request(golden[:-1]) is None
    assert request(packet(2, 0)) is None


def test_duplicate_expiry_and_reboot():
    leases = Leases(0, 0x78563412)
    new, ack = leases.begin(2, leases.token, 0, 1)
    assert new and ack[6] == Result.ACCEPTED
    assert leases.begin(2, leases.token, 1, 1) == (False, ack)
    assert leases.begin(1, leases.token, 1, 1)[1][6] == Result.BAD_REQUEST
    old_token = leases.token
    done = leases.finish(Result.DONE, 2, 2)
    assert leases.begin(2, old_token, 3, 2) == (False, done)
    assert leases.token != old_token
    unused = leases.token
    leases.expire(8)
    assert leases.begin(2, unused, 8, 2)[1][6] == Result.STALE_TOKEN
    reboot = Leases(8, 123)
    assert reboot.begin(2, old_token, 8, 1)[1][6] == Result.STALE_TOKEN


def test_cache_bounded_and_token_wrap():
    leases = Leases(0, 0xFFFFFFFF)
    leases.expire(5)
    assert leases.token == 1
    for n in range(100):
        assert leases.begin(1, leases.token, n, 1)[0]
        leases.finish(Result.DONE, 1, n)
    assert len(leases.cache) <= 64
