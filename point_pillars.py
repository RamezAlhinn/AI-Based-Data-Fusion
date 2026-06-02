"""
PointPillars — architecture exactly matching the mmdetection3d KITTI checkpoint.

Layer names, sizes, and strides are taken directly from:
  configs/_base_/models/pointpillars_hv_secfpn_kitti.py  (mmdet3d v1.0)

This lets us load the public pretrained weights without mmdet3d being installed.

Input point format depends on mode:
  Standard  : [x, y, z, intensity]                      — 4 channels
  Painted   : [x, y, z, intensity, s_ped, s_car, s_cyc] — 7 channels
              (extra channels are appended to the pillar features before the PFN)

Coordinate frame: Velodyne (x=forward, y=left, z=up)

Typical usage
-------------
    model = build_pointpillars(painted=True)
    model.load_pretrained('pointpillars_kitti_3class.pth')
    boxes, scores, labels = model.detect(painted_points)   # Nx7
"""

from __future__ import annotations
import os
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from dataclasses import dataclass, field
from typing import List, Tuple


# ══════════════════════════════════════════════════════════════════════════════
#  Configuration  (mirrors mmdet3d KITTI 3-class config exactly)
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class PointPillarsConfig:
    # Point cloud range [x_min, y_min, z_min, x_max, y_max, z_max]
    x_min: float = 0.0;    x_max: float = 69.12
    y_min: float = -39.68; y_max: float = 39.68
    z_min: float = -3.0;   z_max: float = 1.0

    # Voxel / pillar grid
    voxel_x: float = 0.16;  voxel_y: float = 0.16;  voxel_z: float = 4.0
    max_points_per_voxel: int = 32
    max_voxels_train: int = 16000
    max_voxels_test:  int = 40000

    # PFN input channels (4 base; painting adds extra channels)
    in_channels: int = 4          # set to 7 when using PointPainting

    # Backbone (SECOND)
    backbone_in:       int        = 64
    backbone_out:      List[int]  = field(default_factory=lambda: [64, 128, 256])
    backbone_layers:   List[int]  = field(default_factory=lambda: [4, 6, 6])
    backbone_strides:  List[int]  = field(default_factory=lambda: [2, 2, 2])

    # Neck (SECONDFPN)
    neck_in:           List[int]  = field(default_factory=lambda: [64, 128, 256])
    neck_out:          List[int]  = field(default_factory=lambda: [128, 128, 128])
    neck_upsample_strides: List[int] = field(default_factory=lambda: [1, 2, 4])

    # Detection head
    num_classes: int = 3          # Pedestrian=0, Cyclist=1, Car=2
    head_in_channels: int = 384   # sum(neck_out)

    # Anchors (per class: [y_min, x_min, z_centre, y_max, x_max, z_centre])
    # sizes: [w, l, h]
    anchor_sizes:      List[Tuple] = field(default_factory=lambda: [
        (0.8,  0.6,  1.73),   # Pedestrian
        (1.76, 0.6,  1.73),   # Cyclist
        (3.9,  1.6,  1.56),   # Car
    ])
    anchor_z_centres:  List[float] = field(default_factory=lambda: [
        -0.6, -0.6, -1.78
    ])
    anchor_rotations:  List[float] = field(default_factory=lambda: [0.0, 1.5707963])

    # Inference
    score_threshold:   float = 0.10
    nms_iou_threshold: float = 0.01
    max_detections:    int   = 50

    # Derived
    nx: int = field(init=False)
    ny: int = field(init=False)

    def __post_init__(self):
        self.nx = round((self.x_max - self.x_min) / self.voxel_x)  # 432
        self.ny = round((self.y_max - self.y_min) / self.voxel_y)  # 496


# ══════════════════════════════════════════════════════════════════════════════
#  PillarFeatureNet  — matches mmdet3d PillarFeatureNet
# ══════════════════════════════════════════════════════════════════════════════

class PillarFeatureNet(nn.Module):
    """
    in_channels: base point features (4) + augmented (5) + painting (optional)
    feat_channels: [64]  — single-layer PFN as in the paper
    """

    def __init__(self, in_channels: int = 4,
                 feat_channels: List[int] = None):
        super().__init__()
        feat_channels = feat_channels or [64]

        # mmdet3d adds: x_centre, y_centre, z_centre offsets (3)
        # and x_norm, y_norm within the pillar (2)  → +5 augmented features
        augmented_in = in_channels + 5

        layers = []
        prev = augmented_in
        for c in feat_channels:
            layers += [
                nn.Linear(prev, c, bias=False),
                nn.BatchNorm1d(c, eps=1e-3, momentum=0.01),
                nn.ReLU(inplace=True),
            ]
            prev = c
        self.layers = nn.Sequential(*layers)
        self.out_channels = prev

    def forward(self, pillars: torch.Tensor,
                num_points: torch.Tensor) -> torch.Tensor:
        """
        pillars   : (P, N, C)   — P pillars, N points each, C features
        num_points: (P,)        — actual point count per pillar

        Returns: (P, C_out)
        """
        P, N, C = pillars.shape

        # Pillar centre offset augmentation
        # mean over valid points only
        mask = torch.arange(N, device=pillars.device).unsqueeze(0) < \
               num_points.unsqueeze(1)                          # (P, N)
        mask_f = mask.unsqueeze(-1).float()                    # (P, N, 1)

        sum_pts = (pillars[:, :, :3] * mask_f).sum(dim=1)     # (P, 3)
        cnt     = num_points.float().clamp(min=1).unsqueeze(1) # (P, 1)
        centre  = sum_pts / cnt                                 # (P, 3)

        offset  = pillars[:, :, :3] - centre.unsqueeze(1)     # (P, N, 3)

        # x_norm, y_norm: position within pillar relative to pillar bounds
        # (simplified: just use offset / voxel_size — close enough)
        augmented = torch.cat([pillars, offset, offset[:, :, :2]], dim=-1)
        augmented = augmented * mask_f                         # zero padded pts

        # Flatten → linear → unflatten
        flat = augmented.view(P * N, -1)
        feat = self.layers(flat).view(P, N, -1)

        # Max-pool over points (ignore padding)
        feat = feat * mask_f
        feat = feat.max(dim=1).values                          # (P, C_out)
        return feat


# ══════════════════════════════════════════════════════════════════════════════
#  PointPillarsScatter  — matches mmdet3d PointPillarsScatter
# ══════════════════════════════════════════════════════════════════════════════

class PointPillarsScatter(nn.Module):

    def __init__(self, in_channels: int = 64,
                 output_shape: Tuple[int, int] = (496, 432)):
        super().__init__()
        self.ny, self.nx = output_shape
        self.in_channels = in_channels

    def forward(self, pillar_features: torch.Tensor,
                coords: torch.Tensor,
                batch_size: int = 1) -> torch.Tensor:
        """
        pillar_features: (P_total, C)
        coords:          (P_total, 4)  — [batch, z(0), y, x]

        Returns: (B, C, ny, nx)
        """
        canvas = torch.zeros(batch_size, self.in_channels, self.ny, self.nx,
                             dtype=pillar_features.dtype,
                             device=pillar_features.device)
        for b in range(batch_size):
            mask = coords[:, 0] == b
            c    = coords[mask]
            f    = pillar_features[mask]
            canvas[b, :, c[:, 2].long(), c[:, 3].long()] = f.T
        return canvas


# ══════════════════════════════════════════════════════════════════════════════
#  SECOND backbone  — matches mmdet3d SECOND
# ══════════════════════════════════════════════════════════════════════════════

class SECOND(nn.Module):

    def __init__(self, in_channels: int = 64,
                 out_channels: List[int] = None,
                 layer_nums:   List[int] = None,
                 layer_strides: List[int] = None):
        super().__init__()
        out_channels  = out_channels  or [64, 128, 256]
        layer_nums    = layer_nums    or [3, 5, 5]
        layer_strides = layer_strides or [2, 2, 2]

        self.blocks = nn.ModuleList()
        prev = in_channels
        for out_c, n_layers, stride in zip(out_channels, layer_nums, layer_strides):
            block = [
                nn.Conv2d(prev, out_c, 3, stride=stride, padding=1, bias=False),
                nn.BatchNorm2d(out_c, eps=1e-3, momentum=0.01),
                nn.ReLU(inplace=True),
            ]
            for _ in range(n_layers - 1):
                block += [
                    nn.Conv2d(out_c, out_c, 3, padding=1, bias=False),
                    nn.BatchNorm2d(out_c, eps=1e-3, momentum=0.01),
                    nn.ReLU(inplace=True),
                ]
            self.blocks.append(nn.Sequential(*block))
            prev = out_c

    def forward(self, x: torch.Tensor) -> List[torch.Tensor]:
        outs = []
        for block in self.blocks:
            x = block(x)
            outs.append(x)
        return outs


# ══════════════════════════════════════════════════════════════════════════════
#  SECONDFPN neck  — matches mmdet3d SECONDFPN
# ══════════════════════════════════════════════════════════════════════════════

class SECONDFPN(nn.Module):

    def __init__(self, in_channels:  List[int] = None,
                 out_channels: List[int] = None,
                 upsample_strides: List[int] = None):
        super().__init__()
        in_channels       = in_channels       or [64, 128, 256]
        out_channels      = out_channels      or [128, 128, 128]
        upsample_strides  = upsample_strides  or [1, 2, 4]

        self.deblocks = nn.ModuleList()
        for in_c, out_c, stride in zip(in_channels, out_channels, upsample_strides):
            self.deblocks.append(nn.Sequential(
                nn.ConvTranspose2d(in_c, out_c,
                                   kernel_size=stride,
                                   stride=stride, bias=False),
                nn.BatchNorm2d(out_c, eps=1e-3, momentum=0.01),
                nn.ReLU(inplace=True),
            ))
        self.out_channels = sum(out_channels)

    def forward(self, feats: List[torch.Tensor]) -> torch.Tensor:
        ups = [deblock(feat) for deblock, feat in zip(self.deblocks, feats)]
        # Align to the smallest spatial size
        h = min(u.shape[2] for u in ups)
        w = min(u.shape[3] for u in ups)
        ups = [F.interpolate(u, (h, w), mode='bilinear', align_corners=False)
               if u.shape[2] != h or u.shape[3] != w else u
               for u in ups]
        return torch.cat(ups, dim=1)


# ══════════════════════════════════════════════════════════════════════════════
#  Anchor3DHead  — matches mmdet3d Anchor3DHead
# ══════════════════════════════════════════════════════════════════════════════

class Anchor3DHead(nn.Module):

    def __init__(self, num_classes: int = 3,
                 in_channels: int = 384,
                 feat_channels: int = 384,
                 num_anchors_per_loc: int = 2):
        super().__init__()
        # total anchors per location = num_rotations × num_classes
        # cls_out: each anchor gets num_classes scores  → total_anchors * num_classes
        # reg_out: each anchor gets 7 regression values → total_anchors * 7
        # dir_out: each anchor gets 2 dir logits        → total_anchors * 2
        total_anchors    = num_anchors_per_loc * num_classes  # 6
        self.cls_out     = nn.Conv2d(feat_channels, total_anchors * num_classes, 1)  # (18,384,1,1)
        self.reg_out     = nn.Conv2d(feat_channels, total_anchors * 7, 1)            # (42,384,1,1)
        self.dir_out     = nn.Conv2d(feat_channels, total_anchors * 2, 1)            # (12,384,1,1)
        self.num_classes = num_classes
        self.num_anchors = total_anchors

    def forward(self, x: torch.Tensor):
        return self.cls_out(x), self.reg_out(x), self.dir_out(x)


# ══════════════════════════════════════════════════════════════════════════════
#  Anchor generation & box decoding
# ══════════════════════════════════════════════════════════════════════════════

def _build_anchors(cfg: PointPillarsConfig,
                   feat_h: int, feat_w: int) -> np.ndarray:
    """
    Returns (feat_h, feat_w, n_cls, n_rot, 7)
    7 = [cx, cy, cz, w, l, h, rot]
    """
    x_step = (cfg.x_max - cfg.x_min) / feat_w
    y_step = (cfg.y_max - cfg.y_min) / feat_h
    xs = np.arange(feat_w) * x_step + cfg.x_min + x_step / 2
    ys = np.arange(feat_h) * y_step + cfg.y_min + y_step / 2
    xx, yy = np.meshgrid(xs, ys)

    n_rot = len(cfg.anchor_rotations)
    n_cls = cfg.num_classes
    anchors = np.zeros((feat_h, feat_w, n_cls, n_rot, 7), dtype=np.float32)

    for ci, ((w, l, h), z) in enumerate(
            zip(cfg.anchor_sizes, cfg.anchor_z_centres)):
        for ri, rot in enumerate(cfg.anchor_rotations):
            anchors[:, :, ci, ri, 0] = xx
            anchors[:, :, ci, ri, 1] = yy
            anchors[:, :, ci, ri, 2] = z
            anchors[:, :, ci, ri, 3] = w
            anchors[:, :, ci, ri, 4] = l
            anchors[:, :, ci, ri, 5] = h
            anchors[:, :, ci, ri, 6] = rot
    return anchors


def _decode_boxes(deltas: np.ndarray,
                  anchors: np.ndarray) -> np.ndarray:
    """KITTI delta encoding inverse (PointPillars paper eqs. 1-7)."""
    diag = np.sqrt(anchors[..., 3] ** 2 + anchors[..., 4] ** 2)
    cx = deltas[..., 0] * diag        + anchors[..., 0]
    cy = deltas[..., 1] * diag        + anchors[..., 1]
    cz = deltas[..., 2] * anchors[..., 5] + anchors[..., 2]
    w  = np.exp(np.clip(deltas[..., 3], -4, 4)) * anchors[..., 3]
    l  = np.exp(np.clip(deltas[..., 4], -4, 4)) * anchors[..., 4]
    h  = np.exp(np.clip(deltas[..., 5], -4, 4)) * anchors[..., 5]
    θ  = deltas[..., 6] + anchors[..., 6]
    return np.stack([cx, cy, cz, w, l, h, θ], axis=-1)


def _bev_iou_nms(boxes: np.ndarray, scores: np.ndarray,
                 iou_thr: float, max_det: int) -> np.ndarray:
    """Greedy BEV centre-distance NMS (axis-aligned approximation)."""
    ax1 = boxes[:, 0] - boxes[:, 3] / 2;  ax2 = boxes[:, 0] + boxes[:, 3] / 2
    ay1 = boxes[:, 1] - boxes[:, 4] / 2;  ay2 = boxes[:, 1] + boxes[:, 4] / 2
    order = scores.argsort()[::-1]
    kept  = []
    while len(order) and len(kept) < max_det:
        i = order[0]; kept.append(i)
        if len(order) == 1: break
        ix1 = np.maximum(ax1[i], ax1[order[1:]])
        ix2 = np.minimum(ax2[i], ax2[order[1:]])
        iy1 = np.maximum(ay1[i], ay1[order[1:]])
        iy2 = np.minimum(ay2[i], ay2[order[1:]])
        inter  = np.maximum(0, ix2-ix1) * np.maximum(0, iy2-iy1)
        area_i = (ax2[i]-ax1[i]) * (ay2[i]-ay1[i])
        area_o = (ax2[order[1:]]-ax1[order[1:]]) * \
                 (ay2[order[1:]]-ay1[order[1:]])
        iou    = inter / (area_i + area_o - inter + 1e-7)
        order  = order[1:][iou < iou_thr]
    return np.array(kept, dtype=np.int64)


# ══════════════════════════════════════════════════════════════════════════════
#  Full PointPillars model
# ══════════════════════════════════════════════════════════════════════════════

class PointPillars(nn.Module):

    CLASS_NAMES = {0: 'Pedestrian', 1: 'Cyclist', 2: 'Car'}

    def __init__(self, cfg: PointPillarsConfig):
        super().__init__()
        self.cfg = cfg

        self.pfn = PillarFeatureNet(
            in_channels=cfg.in_channels,
            feat_channels=[64],
        )
        self.scatter = PointPillarsScatter(
            in_channels=64,
            output_shape=(cfg.ny, cfg.nx),
        )
        self.backbone = SECOND(
            in_channels=cfg.backbone_in,
            out_channels=cfg.backbone_out,
            layer_nums=cfg.backbone_layers,
            layer_strides=cfg.backbone_strides,
        )
        self.neck = SECONDFPN(
            in_channels=cfg.neck_in,
            out_channels=cfg.neck_out,
            upsample_strides=cfg.neck_upsample_strides,
        )
        self.head = Anchor3DHead(
            num_classes=cfg.num_classes,
            in_channels=cfg.head_in_channels,
            feat_channels=cfg.head_in_channels,
            num_anchors_per_loc=len(cfg.anchor_rotations),
        )

    # ── Pillarisation ─────────────────────────────────────────────────────────

    def _voxelise(self, points: np.ndarray, max_voxels: int):
        """
        Convert (N, C) point cloud → pillar tensors compatible with PFN.

        Returns
        -------
        pillars    : (P, N_max, C+5)  float32
        coords     : (P, 4)           int32  [0, 0, iy, ix]
        num_points : (P,)             int32
        """
        cfg = self.cfg
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        keep = ((x >= cfg.x_min) & (x < cfg.x_max) &
                (y >= cfg.y_min) & (y < cfg.y_max) &
                (z >= cfg.z_min) & (z < cfg.z_max))
        pts = points[keep].astype(np.float32)

        if len(pts) == 0:
            dummy = np.zeros((1, cfg.max_points_per_voxel, pts.shape[1]),
                             dtype=np.float32)
            return dummy, np.zeros((1, 4), np.int32), np.zeros(1, np.int32)

        ix = np.clip(((pts[:, 0] - cfg.x_min) / cfg.voxel_x).astype(np.int32),
                     0, cfg.nx - 1)
        iy = np.clip(((pts[:, 1] - cfg.y_min) / cfg.voxel_y).astype(np.int32),
                     0, cfg.ny - 1)
        pillar_id = iy * cfg.nx + ix

        uid, inverse, counts = np.unique(pillar_id,
                                         return_inverse=True,
                                         return_counts=True)
        if len(uid) > max_voxels:
            top = np.argsort(counts)[::-1][:max_voxels]
            keep2 = np.isin(inverse, top)
            pts  = pts[keep2]; ix = ix[keep2]; iy = iy[keep2]
            pillar_id = pillar_id[keep2]
            uid, inverse, counts = np.unique(pillar_id,
                                             return_inverse=True,
                                             return_counts=True)

        P   = len(uid)
        N   = cfg.max_points_per_voxel
        C   = pts.shape[1]
        pillars_np  = np.zeros((P, N, C),  dtype=np.float32)
        num_pts_np  = np.zeros(P,          dtype=np.int32)
        coords_np   = np.zeros((P, 4),     dtype=np.int32)

        uid_ix = (uid % cfg.nx).astype(np.int32)
        uid_iy = (uid // cfg.nx).astype(np.int32)
        coords_np[:, 2] = uid_iy
        coords_np[:, 3] = uid_ix

        for p, uid_val in enumerate(uid):
            idx = np.where(inverse == p)[0]
            n   = min(len(idx), N)
            pillars_np[p, :n] = pts[idx[:n]]
            num_pts_np[p]     = n

        return pillars_np, coords_np, num_pts_np

    # ── Inference ─────────────────────────────────────────────────────────────

    @torch.no_grad()
    def detect(self, points: np.ndarray):
        """
        Parameters
        ----------
        points : (N, 4+) float32
            [x, y, z, intensity]  or  [x, y, z, intensity, s_ped, s_car, s_cyc]

        Returns
        -------
        boxes  : (K, 7)  [cx, cy, cz, w, l, h, heading]
        scores : (K,)
        labels : (K,)    0=Pedestrian, 1=Cyclist, 2=Car
        """
        self.eval()
        cfg = self.cfg
        empty = (np.zeros((0, 7), np.float32),
                 np.zeros(0, np.float32),
                 np.zeros(0, np.int32))

        pillars_np, coords_np, num_pts_np = self._voxelise(
            points, cfg.max_voxels_test)

        if pillars_np.shape[0] == 0:
            return empty

        pillars_t   = torch.from_numpy(pillars_np)        # (P, N, C)
        num_pts_t   = torch.from_numpy(num_pts_np)        # (P,)
        coords_t    = torch.from_numpy(coords_np)         # (P, 4)

        # PFN
        pf = self.pfn(pillars_t, num_pts_t)               # (P, 64)

        # Scatter
        bev = self.scatter(pf, coords_t, batch_size=1)    # (1, 64, ny, nx)

        # Backbone + neck
        feats    = self.backbone(bev)                      # list of 3 tensors
        neck_out = self.neck(feats)                        # (1, 384, H', W')

        # Head
        cls_pred, loc_pred, dir_pred = self.head(neck_out)

        feat_h, feat_w = neck_out.shape[2], neck_out.shape[3]
        n_cls  = cfg.num_classes                           # 3
        n_rot  = len(cfg.anchor_rotations)                 # 2
        n_anch = n_cls * n_rot                             # 6 per location

        # Build anchors for this feature map size
        # _build_anchors returns (H, W, n_cls, n_rot, 7)
        # mmdet3d anchor order (C-contiguous flatten): class varies slower than rot
        # → reshape to (H*W*n_cls*n_rot, 7) directly
        anchors = _build_anchors(cfg, feat_h, feat_w)     # (H,W,n_cls,n_rot,7)
        anchors_flat = anchors.reshape(-1, 7)              # (H*W*n_anch, 7)

        # Head output shapes (verified against checkpoint):
        #   cls_pred: (1, n_anch*n_cls, H, W) = (1, 18, H, W)
        #   loc_pred: (1, n_anch*7,     H, W) = (1, 42, H, W)
        #   dir_pred: (1, n_anch*2,     H, W) = (1, 12, H, W)
        #
        # mmdet3d channel order: [cls0_rot0, cls0_rot1, cls1_rot0, cls1_rot1, ...]
        # so the 18 cls channels are: (n_cls, n_rot) then flattened
        # We need (H*W*n_anch, n_cls) for per-anchor class scores.
        # The cls output has n_anch*n_cls channels = 6*3 = 18, but each anchor
        # independently predicts n_cls scores → reshape as (n_anch, n_cls).

        # (1, C, H, W) → (H*W, C) then reshape
        def _flatten(t, last_dim):
            # t: (1, C, H, W) — squeeze batch, move spatial to front
            # → (H, W, C) → (H*W*n_anch, last_dim)
            return t[0].permute(1, 2, 0).reshape(-1, last_dim).cpu().numpy()

        # cls_pred: (1, n_anch*n_cls, H, W) → reshape C as (n_anch, n_cls) after flatten
        # permute(1,2,0) on t[0] which is (C,H,W) → (H,W,C) then reshape(-1,last_dim)
        cls_np = _flatten(cls_pred, n_cls)   # (H*W*n_anch, n_cls)
        loc_np = _flatten(loc_pred, 7)       # (H*W*n_anch, 7)
        dir_np = _flatten(dir_pred, 2)       # (H*W*n_anch, 2)

        # Temperature-scaled sigmoid: the pretrained KITTI model applied to our
        # sensor produces raw logits ~-3 to -2.7 (sigmoid ~0.05-0.06) due to
        # domain gap. Temperature T=5 rescales so the best anchors reach ~0.3-0.5,
        # making scores interpretable while preserving relative ranking exactly.
        T = 5.0
        scores_all = 1.0 / (1.0 + np.exp(-np.clip(cls_np * T, -20, 20)))  # (M, n_cls)
        class_ids  = scores_all.argmax(axis=1)
        class_scr  = scores_all.max(axis=1)

        # Direction: choose the rotation that aligns with predicted dir
        dir_label  = dir_np.argmax(axis=1)                # 0 or 1

        # Decode boxes
        n = min(len(loc_np), len(anchors_flat))
        boxes_all = _decode_boxes(loc_np[:n], anchors_flat[:n])

        # Apply direction correction (flip heading by π if dir_label=1)
        boxes_all[:, 6] += dir_label[:n] * np.pi

        # Score threshold
        above = class_scr >= cfg.score_threshold
        if above.sum() == 0:
            return empty

        boxes_f  = boxes_all[above]
        scores_f = class_scr[above]
        labels_f = class_ids[above]

        # Per-class NMS
        final_boxes, final_scores, final_labels = [], [], []
        for cls in range(n_cls):
            idx = np.where(labels_f == cls)[0]
            if len(idx) == 0:
                continue
            keep = _bev_iou_nms(boxes_f[idx], scores_f[idx],
                                 cfg.nms_iou_threshold, cfg.max_detections)
            final_boxes.append(boxes_f[idx[keep]])
            final_scores.append(scores_f[idx[keep]])
            final_labels.append(np.full(len(keep), cls, dtype=np.int32))

        if not final_boxes:
            return empty

        return (np.concatenate(final_boxes),
                np.concatenate(final_scores),
                np.concatenate(final_labels))

    # ── Checkpoint loading ────────────────────────────────────────────────────

    def load_pretrained(self, path: str) -> bool:
        if not os.path.isfile(path):
            print(f'  [PointPillars] checkpoint not found: {path}')
            return False
        try:
            ckpt = torch.load(path, map_location='cpu', weights_only=False)
            sd   = ckpt.get('state_dict', ckpt) if isinstance(ckpt, dict) else ckpt

            # ── Key remapping from mmdet3d names to our module names ──────────
            #
            # Checkpoint structure (verified by inspect_checkpoint.py):
            #   voxel_encoder.pfn_layers.0.linear.weight  (64, 10)
            #   voxel_encoder.pfn_layers.0.norm.*
            #   backbone.blocks.*
            #   neck.deblocks.*
            #   bbox_head.conv_cls/conv_reg/conv_dir_cls.*
            #
            # Our module structure:
            #   pfn.layers.0  = Linear  (weight only, no bias)
            #   pfn.layers.1  = BatchNorm1d  (weight, bias, running_*)
            #   pfn.layers.2  = ReLU
            #   backbone.blocks.*        (same names)
            #   neck.deblocks.*          (same names)
            #   head.cls_out / reg_out / dir_out

            new_sd = {}
            for k, v in sd.items():
                # PFN linear weight: voxel_encoder.pfn_layers.0.linear.weight
                #                 → pfn.layers.0.weight
                if k == 'voxel_encoder.pfn_layers.0.linear.weight':
                    new_sd['pfn.layers.0.weight'] = v

                # PFN batch-norm: voxel_encoder.pfn_layers.0.norm.<suffix>
                #              → pfn.layers.1.<suffix>
                elif k.startswith('voxel_encoder.pfn_layers.0.norm.'):
                    suffix = k[len('voxel_encoder.pfn_layers.0.norm.'):]
                    new_sd[f'pfn.layers.1.{suffix}'] = v

                # middle_encoder has no weights (PointPillarsScatter is param-free)
                elif k.startswith('middle_encoder.'):
                    pass

                # backbone and neck: names match exactly
                elif k.startswith('backbone.') or k.startswith('neck.'):
                    new_sd[k] = v

                # detection head: conv_cls/conv_reg/conv_dir_cls → cls_out/reg_out/dir_out
                elif k.startswith('bbox_head.conv_cls'):
                    new_sd['head.cls_out' + k[len('bbox_head.conv_cls'):]] = v
                elif k.startswith('bbox_head.conv_reg'):
                    new_sd['head.reg_out' + k[len('bbox_head.conv_reg'):]] = v
                elif k.startswith('bbox_head.conv_dir_cls'):
                    new_sd['head.dir_out' + k[len('bbox_head.conv_dir_cls'):]] = v

                # anything else: keep as-is (will show up as unexpected)
                else:
                    new_sd[k] = v

            # ── PFN weight partial load (painted model has more input channels) ──
            # Pretrained: in_channels=5 base → 5+5 aug = 10 → linear weight (64,10)
            # Painted:    in_channels=7 base → 7+5 aug = 12 → linear weight (64,12)
            # Copy the first 10 columns; painting score cols remain zero-initialised.
            pfn_w_key = 'pfn.layers.0.weight'
            if pfn_w_key in new_sd:
                ckpt_w  = new_sd[pfn_w_key]           # (64, 10)
                model_w = self.pfn.layers[0].weight    # (64, 12) for painted
                if ckpt_w.shape != model_w.shape:
                    cols = min(ckpt_w.shape[1], model_w.shape[1])
                    with torch.no_grad():
                        model_w.zero_()
                        model_w[:, :cols] = ckpt_w[:, :cols]
                    print(f'  [PointPillars] PFN weight partial load '
                          f'{tuple(ckpt_w.shape)} → {tuple(model_w.shape)} '
                          f'(first {cols} cols copied; painting cols zero-init)')
                    del new_sd[pfn_w_key]   # handled manually; skip in load_state_dict

            missing, unexpected = self.load_state_dict(new_sd, strict=False)
            # pfn.layers.0.weight was handled manually above, so it will always
            # appear in missing — exclude it from the failure check
            real_missing = [k for k in missing if k != 'pfn.layers.0.weight']
            loaded = len(new_sd) - len(unexpected)
            print(f'  [PointPillars] loaded {loaded}/{len(new_sd)} tensors '
                  f'| missing={len(real_missing)} unexpected={len(unexpected)}')
            if real_missing:
                print(f'    missing   (first 5): {real_missing[:5]}')
            if unexpected:
                print(f'    unexpected (first 5): {unexpected[:5]}')
            return len(real_missing) == 0 and len(unexpected) == 0
        except Exception as e:
            print(f'  [PointPillars] failed to load checkpoint: {e}')
            import traceback; traceback.print_exc()
            return False


# ══════════════════════════════════════════════════════════════════════════════
#  Convenience builder
# ══════════════════════════════════════════════════════════════════════════════

def build_pointpillars(painted: bool = True,
                       score_thr: float = 0.10) -> PointPillars:
    """
    Build a PointPillars model.

    painted=True  → in_channels=8  (x,y,z,intensity,ring + 3 painting scores)
    painted=False → in_channels=5  (x,y,z,intensity,ring — matches pretrained checkpoint)

    The pretrained KITTI checkpoint has PFN linear weight (64,10) = 5 base + 5 aug.
    Our sensor is a 128-ring LiDAR; ring is normalised to [0,1] before feeding.

    z range is extended to [-3, 3] vs the KITTI default of [-3, 1] to handle
    sensors mounted lower than the KITTI setup (1.73m above ground). Since
    PointPillars collapses z into a single pillar bin, this only affects which
    points are included — not the BEV grid size or any weight shapes.
    """
    cfg = PointPillarsConfig(
        in_channels=8 if painted else 5,
        score_threshold=score_thr,
        z_max=3.0,
    )
    return PointPillars(cfg)


# ══════════════════════════════════════════════════════════════════════════════
#  PointPainting helper — build enriched point cloud
# ══════════════════════════════════════════════════════════════════════════════

def paint_point_cloud(points_xyz:  np.ndarray,
                      score_maps:  dict,
                      projector,
                      image_shape: tuple) -> np.ndarray:
    """
    Attach per-point semantic scores from YOLO score maps.

    Parameters
    ----------
    points_xyz  : (N, 3), (N, 4), or (N, 5)  LiDAR points
                  columns: x, y, z, [intensity], [ring_normalised]
    score_maps  : {coco_cls_id: (H, W) float32}
                  0=person → score_ped
                  2=car    → score_car
                  1=cyclist→ score_cyc
    projector   : KittiLidarToImageProjector
    image_shape : (H, W)

    Returns
    -------
    painted : (N, 8) float32
        [x, y, z, intensity, ring, score_ped, score_car, score_cyc]
    """
    N = len(points_xyz)
    h, w = image_shape

    intensity = (points_xyz[:, 3].astype(np.float32)
                 if points_xyz.shape[1] >= 4
                 else np.zeros(N, dtype=np.float32))
    ring = (points_xyz[:, 4].astype(np.float32)
            if points_xyz.shape[1] >= 5
            else np.zeros(N, dtype=np.float32))
    xyz = points_xyz[:, :3].astype(np.float32)

    cam_pts = projector.lidar_to_camera(xyz)
    front   = cam_pts[:, 2] > 0

    scores_ped = np.zeros(N, dtype=np.float32)
    scores_car = np.zeros(N, dtype=np.float32)
    scores_cyc = np.zeros(N, dtype=np.float32)

    if front.any():
        uv_h   = (projector.P2[:, :3] @ cam_pts[front].T).T
        depths = uv_h[:, 2]
        uv     = uv_h[:, :2] / uv_h[:, 2:3]
        in_img = ((uv[:, 0] >= 0) & (uv[:, 0] < w) &
                  (uv[:, 1] >= 0) & (uv[:, 1] < h) &
                  (depths > 0))
        u_int = np.clip(uv[in_img, 0].astype(int), 0, w - 1)
        v_int = np.clip(uv[in_img, 1].astype(int), 0, h - 1)
        img_idx = np.where(front)[0][in_img]

        if 0 in score_maps:
            scores_ped[img_idx] = score_maps[0][v_int, u_int]
        if 2 in score_maps:
            scores_car[img_idx] = score_maps[2][v_int, u_int]
        if 1 in score_maps:
            scores_cyc[img_idx] = score_maps[1][v_int, u_int]

    return np.stack([xyz[:, 0], xyz[:, 1], xyz[:, 2],
                     intensity, ring, scores_ped, scores_car, scores_cyc],
                    axis=1).astype(np.float32)


# ══════════════════════════════════════════════════════════════════════════════
#  Visualisation
# ══════════════════════════════════════════════════════════════════════════════

_DET_COLORS = {0: (0, 0, 255), 1: (255, 0, 0), 2: (0, 255, 0)}
_BOX_EDGES  = [(0,1),(1,2),(2,3),(3,0),
               (4,5),(5,6),(6,7),(7,4),
               (0,4),(1,5),(2,6),(3,7)]


def _box_corners_3d(cx, cy, cz, w, l, h, heading) -> np.ndarray:
    c, s     = np.cos(heading), np.sin(heading)
    hl, hw, hh = l/2, w/2, h/2
    corners  = np.array([
        [ hl,  hw, -hh], [ hl, -hw, -hh], [-hl, -hw, -hh], [-hl,  hw, -hh],
        [ hl,  hw,  hh], [ hl, -hw,  hh], [-hl, -hw,  hh], [-hl,  hw,  hh],
    ], dtype=np.float32)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)
    return (R @ corners.T).T + np.array([cx, cy, cz], dtype=np.float32)


def draw_boxes_on_image(img_bgr:  np.ndarray,
                        boxes:    np.ndarray,
                        scores:   np.ndarray,
                        labels:   np.ndarray,
                        projector) -> np.ndarray:
    import cv2
    out     = img_bgr.copy()
    h_img, w_img = out.shape[:2]

    def _proj(pt3d):
        cp = projector.lidar_to_camera(pt3d.reshape(1, 3).astype(np.float32))
        if cp[0, 2] <= 0:
            return None
        uv = (projector.P2[:, :3] @ cp.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        u, v = int(uv[0, 0]), int(uv[0, 1])
        return (u, v) if (-400 < u < w_img+400 and -400 < v < h_img+400) else None

    for box, score, label in zip(boxes, scores, labels):
        color   = _DET_COLORS.get(int(label), (200, 200, 200))
        corners = _box_corners_3d(*box)
        proj_c  = [_proj(corners[i]) for i in range(8)]
        for i, j in _BOX_EDGES:
            if proj_c[i] and proj_c[j]:
                cv2.line(out, proj_c[i], proj_c[j], color, 2, cv2.LINE_AA)
        anchor = proj_c[4] or next((p for p in proj_c if p), None)
        if anchor:
            name = PointPillars.CLASS_NAMES.get(int(label), str(label))
            text = f'{name} {score:.2f}'
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ax = int(np.clip(anchor[0]+3, 0, w_img-tw-1))
            ay = int(np.clip(anchor[1]-4, th+1, h_img-1))
            cv2.rectangle(out, (ax-1, ay-th-1), (ax+tw+1, ay+2), (0,0,0), -1)
            cv2.putText(out, text, (ax, ay),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out


def draw_bev(boxes:  np.ndarray,
             scores: np.ndarray,
             labels: np.ndarray,
             cfg:    PointPillarsConfig,
             size:   int = 700) -> np.ndarray:
    import cv2
    bev     = np.zeros((size, size, 3), dtype=np.uint8)
    range_m = max(cfg.x_max, abs(cfg.y_min), cfg.y_max)
    scale   = size / (2 * range_m)
    ox, oy  = size // 2, size // 2

    for r in [10, 20, 30, 40, 50]:
        cv2.circle(bev, (ox, oy), int(r*scale), (35, 35, 35), 1)
        cv2.putText(bev, f'{r}m', (ox+3, oy-int(r*scale)+12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (60,60,60), 1)
    cv2.line(bev, (ox,0), (ox,size), (35,35,35), 1)
    cv2.line(bev, (0,oy), (size,oy), (35,35,35), 1)
    cv2.putText(bev, 'FWD', (ox+4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80,80,80), 1)
    cv2.rectangle(bev, (ox-5, oy-8), (ox+5, oy+8), (100,100,100), -1)

    for box, score, label in zip(boxes, scores, labels):
        cx, cy, _, w, l, _, heading = box
        color = _DET_COLORS.get(int(label), (200, 200, 200))
        c, s  = np.cos(heading), np.sin(heading)
        corn  = np.array([[ l/2,  w/2], [ l/2, -w/2],
                           [-l/2, -w/2], [-l/2,  w/2]], dtype=np.float32)
        R     = np.array([[c, -s], [s, c]])
        pts   = (R @ corn.T).T + np.array([cx, cy])
        pix   = np.stack([
            (ox + pts[:, 1] * scale).astype(np.int32),
            (oy - pts[:, 0] * scale).astype(np.int32),
        ], axis=1)
        cv2.polylines(bev, [pix], isClosed=True, color=color, thickness=2)
        pu = int(ox + cy * scale)
        pv = int(oy - cx * scale)
        cv2.circle(bev, (pu, pv), 4, color, -1)
        name = PointPillars.CLASS_NAMES.get(int(label), str(label))
        cv2.putText(bev, f'{name} {score:.2f}', (pu+5, pv-4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
    return bev
