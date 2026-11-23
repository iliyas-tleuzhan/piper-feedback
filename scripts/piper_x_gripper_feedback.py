"""Effort-limited position resistance, not a NEXT gripper torque estimate."""
import math


def evaluate(master, slave, cfg, active=False):
    """Return a bounded command and gate reason. Caller guarantees freshness."""
    if len(master) != 8 or len(slave) != 8:
        return bytes(8), 'invalid frame'
    if master[6] & 0x3f or slave[6] & 0x3f or not slave[6] & 0x40:
        return bytes(8), 'fault or follower disabled'
    qm = int.from_bytes(master[:4], 'big', signed=True)
    qs = int.from_bytes(slave[:4], 'big', signed=True)
    effort = abs(int.from_bytes(slave[4:6], 'big', signed=True))/1000
    if not 0 <= qm <= cfg['max_opening_um'] or not 0 <= qs <= cfg['max_opening_um']:
        return bytes(8), 'opening out of range'
    gap = cfg.get('release_gap_um', cfg['gap_um']) if active else cfg['gap_um']
    if abs(qs-qm) <= gap:
        return bytes(8), 'opening matched'
    if effort <= cfg['deadband_nm']:
        return bytes(8), 'no load'
    limit = min(cfg['max_effort_nm'], cfg['gain']*(effort-cfg['deadband_nm']))
    # Preserve exact raw-unit differences despite floating-point subtraction.
    units = math.floor(limit*1000 + 1e-9)
    if units <= 0:
        return bytes(8), 'below command resolution'
    return (qs.to_bytes(4,'big',signed=True)+units.to_bytes(2,'big')+bytes([1,0]),
            'resisting')


def resistance(master, slave, cfg):
    """Stateless compatibility helper; caller guarantees fresh feedback."""
    return evaluate(master, slave, cfg)[0]


def validate(cfg):
    for key in ('max_opening_um','gap_um','deadband_nm','max_effort_nm','gain','timeout'):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('Invalid gripper feedback '+key)
    if cfg['max_effort_nm'] > 5 or cfg['gap_um'] >= cfg['max_opening_um']:
        raise ValueError('Invalid gripper effort/opening bounds')
    release = cfg.get('release_gap_um', cfg['gap_um'])
    if not math.isfinite(release) or not 0 <= release <= cfg['gap_um']:
        raise ValueError('Invalid gripper release_gap_um')
