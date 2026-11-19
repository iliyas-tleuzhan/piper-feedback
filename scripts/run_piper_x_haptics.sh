#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
if [[ "${1:-monitor}" == check ]]; then
  exec .venv/bin/python scripts/piper_x_haptics.py --check --config config/piper_x_simulate_torque.yaml
fi
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
case "${1:-monitor}" in
  monitor) exec python scripts/piper_x_haptics.py ;;
  control) exec python scripts/run_piper_x_auto.py ;;
  native) exec python scripts/run_piper_x_auto.py --native ;;
  torque-feedback) exec python scripts/run_piper_x_auto.py --experimental-torque-master ;;
  feedback) exec python scripts/run_piper_x_auto.py --feedback ;;
  infer) exec ros2 run factr2_next next_infer ;;
  dashboard) exec ros2 run factr2_next next_visualize ;;
  *) echo 'Usage: bash scripts/run_piper_x_haptics.sh {check|monitor|control|feedback|native|torque-feedback|infer|dashboard}'; exit 2 ;;
esac
