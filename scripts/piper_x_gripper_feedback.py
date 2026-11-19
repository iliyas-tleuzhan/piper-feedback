"""Effort-limited position resistance, not a NEXT gripper torque estimate."""
import math


def resistance(master, slave, cfg):
    """Return a 0x159 command or disable. Caller guarantees fresh feedback."""
    if master[6] & 0x3f or slave[6] & 0x3f or not slave[6] & 0x40:
        return bytes(8)
    qm = int.from_bytes(master[:4], 'big', signed=True)
    qs = int.from_bytes(slave[:4], 'big', signed=True)
    effort = abs(int.from_bytes(slave[4:6], 'big', signed=True))/1000
    if not 0 <= qm <= cfg['max_opening_um'] or not 0 <= qs <= cfg['max_opening_um']:
        return bytes(8)
    if abs(qs-qm) < cfg['gap_um'] or effort <= cfg['deadband_nm']:
        return bytes(8)
    limit = min(cfg['max_effort_nm'], cfg['gain']*(effort-cfg['deadband_nm']))
    units = int(limit*1000)
    if units <= 0:
        return bytes(8)
    return qs.to_bytes(4,'big',signed=True)+units.to_bytes(2,'big')+bytes([1,0])


def validate(cfg):
    for key in ('max_opening_um','gap_um','deadband_nm','max_effort_nm','gain','timeout'):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('Invalid gripper feedback '+key)
    if cfg['max_effort_nm'] > 5 or cfg['gap_um'] >= cfg['max_opening_um']:
        raise ValueError('Invalid gripper effort/opening bounds')
