"""
MetricsLoggerNode — collects live tracking data from /frustum/objects and
generates validation plots on shutdown (Ctrl+C or bag end).

Plots saved to /workspace/metrics/<timestamp>/:
  confidence_over_time.png  — confidence score per track across frames
  track_lifetime.png        — how many frames each track was alive
  bev_trajectories.png      — top-down (BEV) path of each track
  validation_summary.png    — key validation metrics as a table image

Run:
    ros2 run frustum_detection metrics_logger_node \
        --ros-args -p output_dir:=/workspace/metrics
"""

import math
import os
import collections
import rclpy
from rclpy.node import Node
from perception_msgs.msg import TrackedObjectArray


CLASS_COLOURS = {
    'Car':        '#2ecc71',
    'Pedestrian': '#e74c3c',
    'Cyclist':    '#3498db',
    'Motorcycle': '#9b59b6',
    'Bus':        '#e67e22',
    'Truck':      '#1abc9c',
}
_FALLBACK_COLOURS = ['#f39c12', '#e74c3c', '#9b59b6', '#1abc9c', '#3498db']


def _track_colour(cls_name: str, tid: int) -> str:
    return CLASS_COLOURS.get(cls_name,
                             _FALLBACK_COLOURS[tid % len(_FALLBACK_COLOURS)])


class MetricsLoggerNode(Node):

    def __init__(self):
        super().__init__('metrics_logger_node')
        self.declare_parameter('output_dir', '/workspace/metrics')

        self._out_dir = self.get_parameter('output_dir').value
        os.makedirs(self._out_dir, exist_ok=True)

        # Per-frame data
        self._frame_index   = 0
        self._frames_active = []          # list of active track count per frame

        # Per-track data  (track_id -> list)
        self._conf_history  = collections.defaultdict(list)   # confidence per frame
        self._speed_history = collections.defaultdict(list)   # speed per frame
        self._frame_nums    = collections.defaultdict(list)   # frame index per sample
        self._positions     = collections.defaultdict(list)   # (x, y) per frame
        self._class_name    = {}                               # track_id -> class name
        self._track_first   = {}
        self._track_last    = {}

        # ID switch detection
        self._prev_ids      = set()
        self._id_switches   = 0
        self._switch_frames = []          # frame indices where switches happened

        self.create_subscription(
            TrackedObjectArray, '/frustum/objects', self._cb, 10)

        self.get_logger().info(
            f'MetricsLoggerNode ready — plots will be saved to {self._out_dir}')

    # ── Live data collection ──────────────────────────────────────────────────

    def _cb(self, msg: TrackedObjectArray) -> None:
        self._frame_index += 1
        fi = self._frame_index
        current_ids = set()

        for obj in msg.objects:
            tid = obj.track_id
            current_ids.add(tid)
            self._class_name[tid] = obj.class_name

            vx, vy, vz = obj.velocity.x, obj.velocity.y, obj.velocity.z
            speed = math.sqrt(vx**2 + vy**2 + vz**2)

            self._conf_history[tid].append(obj.confidence)
            self._speed_history[tid].append(speed)
            self._frame_nums[tid].append(fi)
            self._positions[tid].append((obj.position.x, obj.position.y))

            if tid not in self._track_first:
                self._track_first[tid] = fi
            self._track_last[tid] = fi

        self._frames_active.append(len(current_ids))

        # ID switch detection
        new_ids = current_ids - self._prev_ids
        if fi > 1 and new_ids and len(current_ids) <= len(self._prev_ids):
            self._id_switches += len(new_ids)
            self._switch_frames.append(fi)
        self._prev_ids = current_ids

    # ── Plot generation ───────────────────────────────────────────────────────

    def _generate_plots(self) -> None:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().error(
                'matplotlib not installed — run: pip install matplotlib')
            return

        if self._frame_index == 0:
            self.get_logger().warn('No data received — no plots generated.')
            return

        tids = sorted(self._track_first.keys())
        self.get_logger().info(
            f'Generating plots for {len(tids)} tracks over {self._frame_index} frames ...')

        # ── 1. Confidence over time ───────────────────────────────────────────
        fig, ax = plt.subplots(figsize=(10, 4))
        for tid in tids:
            cls = self._class_name.get(tid, 'Unknown')
            ax.plot(self._frame_nums[tid], self._conf_history[tid],
                    label=f'{cls} ID:{tid}',
                    color=_track_colour(cls, tid), linewidth=1.8)
        ax.axhline(0.60, color='green', linestyle='--', linewidth=1.0, label='High threshold (0.60)')
        ax.set_xlabel('Frame')
        ax.set_ylabel('Confidence Score')
        ax.set_title('Detection Confidence per Track over Time')
        ax.set_ylim(0, 1.05)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(self._out_dir, 'confidence_over_time.png'), dpi=150)
        plt.close(fig)

        # ── 2. Track lifetime bar chart ───────────────────────────────────────
        fig, ax = plt.subplots(figsize=(max(6, len(tids) * 1.2), 4))
        labels, lifetimes, bar_colours = [], [], []
        for tid in tids:
            cls = self._class_name.get(tid, 'Unknown')
            lifetime = self._track_last[tid] - self._track_first[tid] + 1
            labels.append(f'{cls}\nID:{tid}')
            lifetimes.append(lifetime)
            bar_colours.append(_track_colour(cls, tid))
        bars = ax.bar(labels, lifetimes, color=bar_colours, edgecolor='white', linewidth=0.8)
        for bar, val in zip(bars, lifetimes):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                    str(val), ha='center', va='bottom', fontsize=9)
        ax.set_ylabel('Frames Tracked')
        ax.set_title('Track Lifetime (frames alive)')
        ax.grid(True, axis='y', alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(self._out_dir, 'track_lifetime.png'), dpi=150)
        plt.close(fig)

        # ── 3. BEV Trajectories ───────────────────────────────────────────────
        fig, ax = plt.subplots(figsize=(7, 7))
        for tid in tids:
            cls = self._class_name.get(tid, 'Unknown')
            pts = self._positions[tid]
            if len(pts) < 2:
                continue
            xs, ys = zip(*pts)
            colour = _track_colour(cls, tid)
            ax.plot(xs, ys, color=colour, linewidth=1.8, alpha=0.85)
            ax.plot(xs[0],  ys[0],  'o', color=colour, markersize=6)
            ax.plot(xs[-1], ys[-1], 's', color=colour, markersize=6)
            ax.annotate(f'{cls} ID:{tid}', (xs[-1], ys[-1]),
                        textcoords='offset points', xytext=(5, 5),
                        fontsize=7, color=colour)
        ax.set_xlabel('X (m)  →  forward')
        ax.set_ylabel('Y (m)  →  left')
        ax.set_title('Top-Down (BEV) Track Trajectories\n(circle = start, square = end)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color='grey', linewidth=0.5)
        ax.axvline(0, color='grey', linewidth=0.5)
        fig.tight_layout()
        fig.savefig(os.path.join(self._out_dir, 'bev_trajectories.png'), dpi=150)
        plt.close(fig)

        # ── 4. Validation summary table ───────────────────────────────────────
        fig, ax = plt.subplots(figsize=(8, 3 + len(tids) * 0.5))
        ax.axis('off')
        rows = [
            ['Total frames processed', str(self._frame_index)],
            ['Total unique tracks',    str(len(tids))],
            ['ID switches',            str(self._id_switches)],
        ]
        for tid in tids:
            cls      = self._class_name.get(tid, 'Unknown')
            lifetime = self._track_last[tid] - self._track_first[tid] + 1
            confs    = self._conf_history[tid]
            mean_c   = sum(confs) / len(confs) if confs else 0.0
            speeds   = self._speed_history[tid]
            mean_s   = sum(speeds) / len(speeds) if speeds else 0.0
            rows.append([
                f'{cls} ID:{tid}',
                f'lifetime: {lifetime} frames | mean conf: {mean_c:.2f} | mean speed: {mean_s:.2f} m/s'
            ])
        table = ax.table(
            cellText=rows,
            colLabels=['Metric', 'Value'],
            cellLoc='left', loc='center',
            colWidths=[0.3, 0.7],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 1.5)
        for (r, c), cell in table.get_celld().items():
            if r == 0:
                cell.set_facecolor('#2c3e50')
                cell.set_text_props(color='white', fontweight='bold')
            elif r % 2 == 0:
                cell.set_facecolor('#ecf0f1')
        ax.set_title('Validation Summary', fontsize=12, fontweight='bold', pad=12)
        fig.tight_layout()
        fig.savefig(os.path.join(self._out_dir, 'validation_summary.png'), dpi=150)
        plt.close(fig)

        self.get_logger().info(f'Plots saved to: {self._out_dir}')

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._generate_plots()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
