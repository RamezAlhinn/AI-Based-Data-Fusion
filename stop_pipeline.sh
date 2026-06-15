#!/usr/bin/env bash
# stop_pipeline.sh — Stop all pipeline processes started by start_pipeline.sh

PID_FILE=/workspace/log/.pipeline_pids

if [[ -f "$PID_FILE" ]]; then
  PIDS=$(cat "$PID_FILE")
  echo "[pipeline] Stopping PIDs: $PIDS"
  kill $PIDS 2>/dev/null && echo "[pipeline] Done." || echo "[pipeline] Some processes already stopped."
  rm -f "$PID_FILE"
else
  echo "[pipeline] No PID file found — killing by name..."
  pkill -f "ros2 bag play" 2>/dev/null
  pkill -f "painting_node" 2>/dev/null
  pkill -f "frustum_node" 2>/dev/null
  pkill -f "foxglove_bridge" 2>/dev/null
  echo "[pipeline] Done."
fi
