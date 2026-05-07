#!/usr/bin/env bash
# Make Yanshee speak text via TTS.
# Usage:  ./say.sh "Hello world"
#         ROBOT_IP=10.73.35.187 ./say.sh "I am ready"

set -euo pipefail

TEXT="${1:?usage: $0 \"text to speak\"}"
IP="${ROBOT_IP:-10.73.35.187}"

curl -m 10 -sS -X PUT "http://$IP:9090/v1/voice/tts" \
  -H 'Content-Type: application/json' \
  -d "{\"tts\":\"$TEXT\",\"interrupt\":true,\"timestamp\":0}"
echo
