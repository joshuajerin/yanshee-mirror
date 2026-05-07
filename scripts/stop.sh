#!/usr/bin/env bash
# Emergency stop — halt all motion immediately.
# Usage:  ./stop.sh

set -euo pipefail
IP="${ROBOT_IP:-10.73.35.187}"

curl -m 5 -sS -X PUT "http://$IP:9090/v1/motions" \
  -H 'Content-Type: application/json' \
  -d '{"operation":"stop","motion":{"name":"","repeat":1,"speed":"normal"},"timestamp":0,"version":"v1"}'
echo
