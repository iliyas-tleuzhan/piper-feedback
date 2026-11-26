#!/usr/bin/env python3
"""Supervise continuous NEXT torque reflection or native observation-only teleop."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def check_can_queues():
    """Read interface configuration; never change it or send a CAN frame."""
    for name in ('can_master', 'can_slave'):
        result = subprocess.run(['ip', '-details', '-json', 'link', 'show', 'dev', name],
                                check=True, capture_output=True, text=True)
        link = json.loads(result.stdout)[0]
        info = link.get('linkinfo', {}).get('info_data', {})
        if ('UP' not in link.get('flags', []) or info.get('state') != 'ERROR-ACTIVE'
                or info.get('bittiming', {}).get('bitrate') != 1000000):
            raise RuntimeError(name+' must be UP, ERROR-ACTIVE at 1000000 bit/s')
        if link.get('txqlen', 0) < 64:
            raise RuntimeError(name+' TX queue is too small. Run: sudo bash scripts/setup_piper_x_can.sh')


def terminate(child):
    if child is None or child.poll() is not None:
        return
    child.terminate()
    try:
        child.wait(timeout=4)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--native', action='store_true', help='Fixed roles; NEXT observation only')
    modes.add_argument('--feedback', action='store_true', help='Continuous NEXT torque reflection; changes both firmware roles to slave')
    modes.add_argument('--experimental-torque-master', action='store_true',
                       help='Alias for continuous torque feedback')
    args = parser.parse_args(argv)
    return args


def main():
    args = parse_args()
    config = ROOT/('config/piper_x_haptics.yaml' if args.native else 'config/piper_x_simulate_torque.yaml')
    subprocess.run([sys.executable, str(ROOT/'scripts/piper_x_haptics.py'), '--check', '--config', str(config)], check=True)
    if not args.native:
        check_can_queues()
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    inference = controller = bridge = trace = None
    log_path = ROOT/'logs/piper_x_next.log'
    log_path.parent.mkdir(exist_ok=True)
    try:
        with log_path.open('a') as log:
            inference = subprocess.Popen([sys.executable, '-u', '-c',
                'from factr2_next.inference.inference_node import main; main()',
                '--ros-args', '-p', 'config_file:='+str(ROOT/'src/factr2_next/factr2_next/config/inference.yaml')],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True, cwd=ROOT)
            if not args.native:
                # Torque controller publishes the trained NEXT input stream itself.
                # A second bridge here would interleave incompatible samples.
                command = [sys.executable, str(ROOT/'scripts/piper_x_haptics.py'),
                           '--control', '--continuous-torque', '--config', str(ROOT/'config/piper_x_simulate_torque.yaml')]
                print('CONTINUOUS TORQUE: can_master measured motion -> can_slave; NEXT can_slave torque -> can_master MIT. '
                      'Startup sets BOTH firmware roles to slave for PC control; can_master is the software leader. '
                      'No runtime role swaps. Support both arms during startup. Ctrl-C requests holds.', flush=True)
            else:
                bridge = subprocess.Popen([sys.executable, str(ROOT/'scripts/piper_can_record_bridge.py'), '--can', 'can_slave'],
                                          stdout=log, stderr=subprocess.STDOUT, start_new_session=True, cwd=ROOT)
                command = [sys.executable, str(ROOT/'scripts/piper_x_native_teleop.py')]
                print('Native master mode; NEXT estimates only (no force reflection).', flush=True)
            controller = subprocess.Popen(command,
                                          start_new_session=True, cwd=ROOT)
            print('NEXT log: '+str(log_path), flush=True)
            while controller.poll() is None:
                if inference.poll() is not None:
                    raise RuntimeError('NEXT exited; stopping teleop. See '+str(log_path))
                if bridge is not None and bridge.poll() is not None:
                    raise RuntimeError('NEXT input bridge exited; stopping teleop. See '+str(log_path))
                time.sleep(.1)
            return controller.returncode
    except KeyboardInterrupt:
        return 0
    finally:
        # Let the controller disable joints before stopping inference.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        terminate(controller)
        terminate(trace)
        terminate(bridge)
        terminate(inference)


if __name__ == '__main__':
    raise SystemExit(main())
