#!/usr/bin/env bash
# start_pipeline.sh — Start the full PointPainting + Frustum Detection stack.
#
# Usage:
#   bash /workspace/start_pipeline.sh
#
# All output is logged to /workspace/log/*.log
# Stop everything with:  bash /workspace/stop_pipeline.sh

set -e
WS=/workspace/ros2_ws
LOG=/workspace/log
BAG=/workspace/studentProject1
CALIB=/workspace/calib.txt
CKPT=/workspace/models/yolo11n-seg.pt

source "$WS/install/setup.bash"

mkdir -p "$LOG"

echo "[pipeline] Starting ros2 bag play..."
nohup ros2 bag play "$BAG" --loop > "$LOG/bag_play.log" 2>&1 &
BAG_PID=$!
echo "  bag PID=$BAG_PID"

echo "[pipeline] Starting Foxglove bridge on port 9090..."
nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090 \
  > "$LOG/foxglove.log" 2>&1 &
FOX_PID=$!
echo "  foxglove PID=$FOX_PID"

echo "[pipeline] Starting painting_node (YOLO load ~4s)..."
nohup ros2 run point_painting painting_node --ros-args \
  -p calib_file:="$CALIB" \
  -p checkpoint_path:="$CKPT" \
  > "$LOG/painting_node_live.log" 2>&1 &
PAINT_PID=$!
echo "  painting_node PID=$PAINT_PID"

echo "[pipeline] Starting frustum_node..."
nohup ros2 run frustum_detection frustum_node --ros-args \
  -p calib_file:="$CALIB" \
  > "$LOG/frustum_node_live.log" 2>&1 &
FRUS_PID=$!
echo "  frustum_node PID=$FRUS_PID"

# Save PIDs for stop script
echo "$BAG_PID $FOX_PID $PAINT_PID $FRUS_PID" > "$LOG/.pipeline_pids"

echo ""
echo "[pipeline] All processes started. Waiting ~12s for nodes to initialise..."
sleep 12

echo ""
echo "[pipeline] Status:"
echo "  bag_play:       $(tail -1 $LOG/bag_play.log 2>/dev/null | head -c 80)"
echo "  painting_node:  $(tail -1 $LOG/painting_node_live.log 2>/dev/null | head -c 80)"
echo "  frustum_node:   $(tail -1 $LOG/frustum_node_live.log 2>/dev/null | head -c 80)"
echo ""
echo "[pipeline] Connect Foxglove → ws://localhost:9090"
echo "  Subscribe to: /frustum/markers  (3D boxes + track IDs)"
echo "                /frustum/bev       (camera + BEV panel)"
echo "                /painting/painted_cloud (coloured point cloud)"
echo ""
echo "[pipeline] Logs:"
echo "  tail -f $LOG/painting_node_live.log"
echo "  tail -f $LOG/frustum_node_live.log"
