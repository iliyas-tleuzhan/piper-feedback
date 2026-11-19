#!/usr/bin/env bash
# Configure both Piper X SocketCAN interfaces. Stop controllers before running.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Run: sudo bash scripts/bringup_piper_x_can.sh" >&2
  exit 1
fi
command -v ip >/dev/null || { echo 'Missing ip command (iproute2).' >&2; exit 1; }

# Check both adapters exist before changing either interface.
for channel in can0 can1; do
  if ! ip link show dev "$channel" >/dev/null 2>&1; then
    echo "Missing interface: $channel. Connect both CAN adapters first." >&2
    exit 1
  fi
done

for channel in can0 can1; do
  ip link set dev "$channel" down
  ip link set dev "$channel" type can bitrate 1000000
  ip link set dev "$channel" txqueuelen 64
  ip link set dev "$channel" up
done

for channel in can0 can1; do
  ip -details link show dev "$channel"
done
echo 'can0 and can1 are UP at 1 Mbit/s with TX queues of 64. No motor commands sent.'
