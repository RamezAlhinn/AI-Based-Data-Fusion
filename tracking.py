"""
tracking.py — lightweight 3D SORT multi-object tracker

Requires only numpy and scipy (linear_sum_assignment).

State vector per track  : [cx, cy, cz, heading, vx, vy, vz]
Measurement vector      : [cx, cy, cz, heading]
Association metric      : 2-D BEV centre distance (metres)
"""

from __future__ import annotations
import numpy as np
from scipy.optimize import linear_sum_assignment
import colorsys
from typing import List, Optional

# ── Colour palette ─────────────────────────────────────────────────────────────

def _id_to_bgr(track_id: int) -> tuple[int, int, int]:
    """Map a track ID to a visually distinct, stable BGR colour."""
    hue = (track_id * 137.508) % 360.0          # golden-angle spread
    r, g, b = colorsys.hsv_to_rgb(hue / 360.0, 0.85, 0.95)
    return (int(b * 255), int(g * 255), int(r * 255))   # BGR


# ── Kalman filter (constant-velocity) ─────────────────────────────────────────

class KalmanBox3D:
    """
    Tracks a single 3-D bounding box with a constant-velocity Kalman filter.

    State  x = [cx, cy, cz, heading, vx, vy, vz]
    Measurement z = [cx, cy, cz, heading]
    """

    _F = None  # transition matrix (shared)
    _H = None  # measurement matrix (shared)

    def __init__(self, box: np.ndarray):
        """
        box: [cx, cy, cz, w, l, h, heading]
        """
        if KalmanBox3D._F is None:
            # State transition: constant velocity
            F = np.eye(7, dtype=np.float64)
            for i in range(4):
                F[i, i + 3] = 1.0        # cx += vx * dt (dt=1 frame)
            KalmanBox3D._F = F
            # Measurement: observe position + heading only
            H = np.zeros((4, 7), dtype=np.float64)
            H[0, 0] = H[1, 1] = H[2, 2] = H[3, 3] = 1.0
            KalmanBox3D._H = H

        # State & covariance
        self.x = np.zeros((7, 1), dtype=np.float64)
        self.x[:4, 0] = box[[0, 1, 2, 6]]    # [cx, cy, cz, heading]

        self.P = np.eye(7, dtype=np.float64)
        self.P[4:, 4:] *= 100.0   # high uncertainty on velocity at birth

        # Process noise Q
        self.Q = np.eye(7, dtype=np.float64)
        self.Q[:3, :3] *= 0.1     # position
        self.Q[3, 3] = 0.05       # heading
        self.Q[4:, 4:] *= 1.0     # velocity

        # Measurement noise R
        self.R = np.eye(4, dtype=np.float64)
        self.R[:3, :3] *= 0.5     # position noise
        self.R[3, 3] = 0.5        # heading noise

        # Keep the full box dimensions from the latest detection
        self.dims = box[3:6].copy()  # [w, l, h]

    def predict(self) -> np.ndarray:
        """Predict next state. Returns [cx, cy, cz, w, l, h, heading]."""
        self.x = KalmanBox3D._F @ self.x
        self.P = KalmanBox3D._F @ self.P @ KalmanBox3D._F.T + self.Q
        return self._to_box()

    def update(self, box: np.ndarray):
        """Correct state with a new measurement box [cx,cy,cz,w,l,h,heading]."""
        z = box[[0, 1, 2, 6]].reshape(4, 1).astype(np.float64)
        S = KalmanBox3D._H @ self.P @ KalmanBox3D._H.T + self.R
        K = self.P @ KalmanBox3D._H.T @ np.linalg.inv(S)
        self.x += K @ (z - KalmanBox3D._H @ self.x)
        self.P = (np.eye(7) - K @ KalmanBox3D._H) @ self.P
        # Always use the measured box dimensions (more reliable than estimated)
        self.dims = box[3:6].copy()

    def _to_box(self) -> np.ndarray:
        s = self.x[:, 0]
        return np.array([s[0], s[1], s[2],
                         self.dims[0], self.dims[1], self.dims[2],
                         s[3]], dtype=np.float32)


# ── Track object ───────────────────────────────────────────────────────────────

class Track:
    _next_id = 1

    def __init__(self, box: np.ndarray, score: float, label: int):
        self.id    = Track._next_id
        Track._next_id += 1
        self.label = label
        self.score = score
        self.color = _id_to_bgr(self.id)
        self.kf    = KalmanBox3D(box)
        self.box   = box.copy()            # current best box
        self.history: list[np.ndarray] = [box[:3].copy()]  # trail of centres

        self.hits        = 1
        self.age         = 1
        self.time_since_update = 0
        self.confirmed   = False           # True after min_hits

    def predict(self):
        self.box = self.kf.predict()
        self.age += 1
        self.time_since_update += 1

    def update(self, box: np.ndarray, score: float):
        self.score = score
        self.kf.update(box)
        self.box = self.kf._to_box()
        self.hits += 1
        self.time_since_update = 0
        self.history.append(self.box[:3].copy())
        if len(self.history) > 20:
            self.history.pop(0)

    @property
    def bev_center(self) -> np.ndarray:
        return self.box[:2]


# ── SORT tracker ───────────────────────────────────────────────────────────────

class Sort3D:
    """
    Simple Online and Realtime 3-D Tracking (adapted from SORT).

    Parameters
    ----------
    max_age   : frames a track survives without a matching detection
    min_hits  : detections needed before a track is confirmed/returned
    max_dist  : maximum BEV centre distance (m) for a valid association
    """

    def __init__(self, max_age: int = 5, min_hits: int = 2, max_dist: float = 3.0):
        self.max_age  = max_age
        self.min_hits = min_hits
        self.max_dist = max_dist
        self.tracks: list[Track] = []

    def update(
        self,
        boxes:  np.ndarray,   # (N, 7)  [cx,cy,cz,w,l,h,heading]
        scores: list[float],
        labels: list[int],
    ) -> list[Track]:
        """
        Run one update step.

        Returns a list of confirmed tracks (may include predicted tracks
        with time_since_update > 0 if they were confirmed earlier).
        """
        # 1. Predict all existing tracks
        for t in self.tracks:
            t.predict()

        # 2. Build cost matrix: BEV distance between each track and each detection
        N_trk = len(self.tracks)
        N_det = len(boxes)

        if N_trk > 0 and N_det > 0:
            trk_centers = np.stack([t.bev_center for t in self.tracks])  # (T,2)
            det_centers = boxes[:, :2]                                    # (D,2)
            diff = trk_centers[:, None, :] - det_centers[None, :, :]     # (T,D,2)
            cost = np.linalg.norm(diff, axis=2)                          # (T,D)

            # Hungarian assignment
            row_ind, col_ind = linear_sum_assignment(cost)

            matched_trk  = set()
            matched_det  = set()
            for r, c in zip(row_ind, col_ind):
                if cost[r, c] <= self.max_dist:
                    self.tracks[r].update(boxes[c], scores[c])
                    matched_trk.add(r)
                    matched_det.add(c)
        else:
            matched_trk = set()
            matched_det = set()

        # 3. Create new tracks for unmatched detections
        for d in range(N_det):
            if d not in matched_det:
                self.tracks.append(Track(boxes[d], scores[d], int(labels[d])))

        # 4. Mark confirmed tracks and prune dead ones
        alive = []
        for t in self.tracks:
            if t.hits >= self.min_hits:
                t.confirmed = True
            if t.time_since_update <= self.max_age:
                alive.append(t)
        self.tracks = alive

        # 5. Return confirmed tracks
        return [t for t in self.tracks if t.confirmed]
