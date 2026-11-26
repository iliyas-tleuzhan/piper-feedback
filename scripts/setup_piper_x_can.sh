#!/usr/bin/env bash
# Increase only the SocketCAN transmit queues. No frames, mode changes or resets.
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
  echo "Run: sudo bash scripts/setup_piper_x_can.sh" >&2
  exit 1
fi
for channel in can_master can_slave; do
  ip link show dev "$channel" >/dev/null
done
for channel in can_master can_slave; do
  ip link set dev "$channel" txqueuelen 64
  ip -details link show dev "$channel"
done
echo 'CAN TX queues set to 64 frames. Bitrate and link state unchanged; no motor commands sent.'
