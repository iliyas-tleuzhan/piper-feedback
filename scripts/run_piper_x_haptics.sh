#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
mode="${1:-feedback}"
case "$mode" in
  teleop)
    exec .venv/bin/python scripts/piper_x_native_teleop.py --config config/piper_x_haptics.yaml ;;
esac
if [[ "$mode" == check ]]; then
  exec .venv/bin/python scripts/piper_x_haptics.py --check --config config/piper_x_simulate_torque.yaml
fi
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
case "$mode" in
  monitor) exec python scripts/piper_x_haptics.py ;;
  native) exec python scripts/run_piper_x_auto.py --native ;;
  torque-feedback) exec python scripts/run_piper_x_auto.py --experimental-torque-master ;;
  feedback|control) exec python scripts/run_piper_x_auto.py --feedback ;;
  infer) exec ros2 run factr2_next next_infer ;;
  dashboard) exec ros2 run factr2_next next_visualize ;;
  *) echo 'Usage: bash scripts/run_piper_x_haptics.sh {teleop|control|check|monitor|feedback|native|torque-feedback|infer|dashboard} (default: feedback, both firmware slaves)'; exit 2 ;;
esac
