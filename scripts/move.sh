#!/usr/bin/env bash
# Fire a Yanshee preset motion by name.
# Usage:  ./move.sh Reset
#         ./move.sh RaiseRightHand
#         ./move.sh Hug normal 1
#         ROBOT_IP=10.73.35.187 ./move.sh PushUp slow 2
#
# Args: <name> [speed=normal] [repeat=1]
# Speeds: very slow | slow | normal | fast | very fast

set -euo pipefail

NAME="${1:?usage: $0 <motion_name> [speed] [repeat]}"
SPEED="${2:-normal}"
REPEAT="${3:-1}"
IP="${ROBOT_IP:-10.73.35.187}"

curl -m 10 -sS -X PUT "http://$IP:9090/v1/motions" \
  -H 'Content-Type: application/json' \
  -d "{\"operation\":\"start\",\"motion\":{\"name\":\"$NAME\",\"repeat\":$REPEAT,\"speed\":\"$SPEED\"},\"timestamp\":0,\"version\":\"v1\"}"
echo
