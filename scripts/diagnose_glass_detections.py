"""
Deep analysis of glass reflection false positives introduced by BN clamping fix.

Checks:
  1. Are detections in the glass region from empty pillars?
  2. What are the model activation patterns (cls, loc, dir logits)?
  3. How sensitive is the model to the BN variance clamping threshold?
  4. Is this a BN fix problem or a training data problem?
"""

import sys
import argparse
import numpy as np
import torch
sys.path.insert(0, '/workspace')

from point_pillars import build_pointpillars, PointPillarsConfig

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint', default='/workspace/models/pointpillars_kitti_3class.pth')
parser.add_argument('--isolation-output', default='/workspace/isolation_output')
args = parser.parse_args()

# ══════════════════════════════════════════════════════════════════════════════
# 1. Load checkpoint and inspect BN variance clamping
# ══════════════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("1. CHECKPOINT BN VARIANCE DISTRIBUTION")
print("="*80)

ckpt = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
sd   = ckpt.get('state_dict', ckpt) if isinstance(ckpt, dict) else ckpt

all_vars = {}
for key in sd.keys():
    if 'running_var' in key:
        var = sd[key].cpu().numpy()
        all_vars[key] = var

print(f"\nFound {len(all_vars)} BatchNorm layers:")
print("\nBN Layer                                      | Min Var   | Mean Var  | Max Var   | <0.05  | <0.01")
print("-" * 110)
for key in sorted(all_vars.keys()):
    var = all_vars[key]
    print(f"{key:<45} | {var.min():>9.2e} | {var.mean():>9.4f} | {var.max():>9.4f} | {(var < 0.05).sum():>5} | {(var < 0.01).sum():>5}")

# ══════════════════════════════════════════════════════════════════════════════
# 2. Find the highest confidence false detection from a test frame
# ══════════════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("2. ANALYZING REAL DETECTION OUTPUT")
print("="*80)

model = build_pointpillars(painted=False)
model.load_pretrained(args.checkpoint)
model.eval()

# Load a recent test frame from isolation_output
import glob
import os
import json

pkl_files = sorted(glob.glob(f'{args.isolation_output}/*.pkl'))
if not pkl_files:
    print(f"No pickle files in {args.isolation_output}")
    sys.exit(1)

latest_pkl = pkl_files[-1]
print(f"\nLoading test frame: {latest_pkl}")

import pickle
try:
    with open(latest_pkl, 'rb') as f:
        data = pickle.load(f)
    points = data['points'].astype(np.float32)
except Exception as e:
    print(f"Error loading {latest_pkl}: {e}")
    print("Skipping real frame analysis...")
    points = None

if points is not None:
    print(f"Point cloud shape: {points.shape}")
    print(f"Point range X: [{points[:,0].min():.2f}, {points[:,0].max():.2f}]")
    print(f"Point range Y: [{points[:,1].min():.2f}, {points[:,1].max():.2f}]")
    
    # Run detection
    with torch.no_grad():
        boxes, scores, labels = model.detect(points)
    
    print(f"\nTotal detections: {len(boxes)}")
    if len(boxes) > 0:
        print("\nTop 10 detections:")
        top_idx = np.argsort(-scores)[:10]
        for rank, idx in enumerate(top_idx, 1):
            box = boxes[idx]
            score = scores[idx]
            label = labels[idx]
            class_name = model.CLASS_NAMES.get(int(label), str(label))
            print(f"  {rank:2d}. {class_name:<12} score={score:.3f} @ ({box[0]:7.2f}, {box[1]:7.2f}, {box[2]:7.2f})")
        
        # Find detections in glass region (right side, x > 25, y > -20)
        glass_region = (boxes[:, 0] > 20) & (boxes[:, 1] > -25) & (boxes[:, 1] < -5)
        glass_dets = np.where(glass_region)[0]
        
        if len(glass_dets) > 0:
            print(f"\n⚠️  Found {len(glass_dets)} detections in glass region (x>20, -25<y<-5):")
            for idx in glass_dets:
                box = boxes[idx]
                score = scores[idx]
                label = labels[idx]
                class_name = model.CLASS_NAMES.get(int(label), str(label))
                print(f"    {class_name:<12} score={score:.3f} @ ({box[0]:7.2f}, {box[1]:7.2f}, {box[2]:7.2f})")
        else:
            print(f"\n✓ No detections in glass region")

# ══════════════════════════════════════════════════════════════════════════════
# 3. Analyze what happens to empty pillars
# ══════════════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("3. EMPTY PILLAR ACTIVATION ANALYSIS")
print("="*80)

cfg = model.cfg

# Create synthetic point cloud with empty regions (like glass)
np.random.seed(42)
# Valid points in foreground area
x_valid = np.random.uniform(5.0, 30.0, 10000)
y_valid = np.random.uniform(-30.0, -10.0, 10000)  # Left side (no glass)
z_valid = np.random.uniform(-1.9, 0.4, 10000)
r_valid = np.random.uniform(0.05, 0.5, 10000)
valid_pts = np.stack([x_valid, y_valid, z_valid, r_valid], axis=1).astype(np.float32)

# Glass region has NO points (or very few)
print(f"Valid points: {len(valid_pts)}")
print(f"Point range X: [{valid_pts[:,0].min():.2f}, {valid_pts[:,0].max():.2f}]")
print(f"Point range Y: [{valid_pts[:,1].min():.2f}, {valid_pts[:,1].max():.2f}]")
print(f"Glass region X: [20, 69.12] (right side)")
print(f"Glass region Y: [-20, 0] (expected in real data)")

with torch.no_grad():
    # Voxelize to see which pillars are empty
    pillars_np, coords_np, num_pts_np = model._voxelise(valid_pts, cfg.max_voxels_test)
    print(f"\nTotal pillars activated: {pillars_np.shape[0]}")
    
    # Check if any glass region pillars have points
    ix_glass = (coords_np[:, 3] >= round(20.0 / cfg.voxel_x))
    iy_glass = (coords_np[:, 2] >= round((cfg.y_min + 20.0) / cfg.voxel_y)) & \
               (coords_np[:, 2] < round((cfg.y_min + 40.0) / cfg.voxel_y))
    glass_pillars_with_points = np.where(ix_glass & iy_glass)[0]
    print(f"Pillars in glass region with points: {len(glass_pillars_with_points)}")
    
    if len(glass_pillars_with_points) > 0:
        pts_in_glass = num_pts_np[glass_pillars_with_points]
        print(f"  Points per glass pillar: min={pts_in_glass.min()}, "
              f"mean={pts_in_glass.mean():.1f}, max={pts_in_glass.max()}")

# ══════════════════════════════════════════════════════════════════════════════
# 4. Test BN clamping sensitivity
# ══════════════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("4. BN VARIANCE CLAMPING SENSITIVITY")
print("="*80)

# Check the current clamping threshold effect
print("\nCurrent threshold in code: BN_VAR_MIN = 0.05")
print("\nBN variance statistics BEFORE clamping:")
print("  Layer                                      | Min    | Mean   | Max    | <0.05  | <0.01  |")
print("-" * 100)

total_clamped = 0
for key in sorted(all_vars.keys()):
    var = all_vars[key]
    n_under_threshold = (var < 0.05).sum()
    total_clamped += n_under_threshold
    print(f"  {key:<40} | {var.min():>6.2e} | {var.mean():>6.4f} | {var.max():>6.4f} | {n_under_threshold:>6} | {(var < 0.01).sum():>6} |")

print(f"\nTotal BN channels clamped: {total_clamped}")
print(f"\n⚠️  Analysis:")
print(f"  - ALL {total_clamped} under-threshold channels get scaled up by 2-100×")
print(f"  - This amplifies signal in ALL regions, including empty pillars")
print(f"  - Empty/invalid regions with random features now have higher scores")
print(f"  - The model may be learning to hallucinate in data-poor regions")

# ══════════════════════════════════════════════════════════════════════════════
# 5. Hypothesis: Lower the threshold
# ══════════════════════════════════════════════════════════════════════════════
print("\n" + "="*80)
print("5. PROPOSED FIX: ANALYZE ALTERNATIVE THRESHOLDS")
print("="*80)

for threshold in [0.01, 0.02, 0.05, 0.10]:
    total_would_clamp = sum((all_vars[k] < threshold).sum().item() for k in all_vars)
    print(f"  Threshold={threshold:>5}: would clamp {total_would_clamp:>5} BN channels")

print("\n⚠️  RECOMMENDATION:")
print("  The 0.05 threshold is very aggressive. Consider:")
print("  1. Lower to 0.02 (clamps 20-40 channels instead of 900+)")
print("  2. Or: Only clamp layers with VAR < 1e-3 (dead neurons)")
print("  3. Or: Use per-layer median variance as threshold (adaptive)")

print("\n" + "="*80)
