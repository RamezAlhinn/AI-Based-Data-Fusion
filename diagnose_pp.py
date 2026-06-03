"""
Deep diagnostic for PointPillars checkpoint alignment.

Run inside the container:
  python3 /workspace/diagnose_pp.py \
      --checkpoint /workspace/pointpillars_kitti_3class.pth

What this checks:
  1. Checkpoint key names and shapes
  2. PFN linear weight statistics (mean/std — should be non-zero)
  3. Synthetic KITTI-like point cloud → are scores healthy?
  4. Real bag frame → pillar stats and score distribution
"""
import sys, argparse, numpy as np, torch
sys.path.insert(0, '/workspace')
from point_pillars import build_pointpillars, PointPillarsConfig

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint', default='/workspace/pointpillars_kitti_3class.pth')
args = parser.parse_args()

# ── 1. Raw checkpoint keys ────────────────────────────────────────────────────
print("\n=== 1. Checkpoint keys ===")
ckpt = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
sd   = ckpt.get('state_dict', ckpt) if isinstance(ckpt, dict) else ckpt
for k, v in sd.items():
    print(f"  {k:<70s} {str(tuple(v.shape)):<25s} {v.dtype}")

# ── 2. PFN weight stats ───────────────────────────────────────────────────────
print("\n=== 2. PFN linear weight stats ===")
pfn_key = 'voxel_encoder.pfn_layers.0.linear.weight'
if pfn_key in sd:
    w = sd[pfn_key].float()
    print(f"  shape : {tuple(w.shape)}")
    print(f"  mean  : {w.mean():.6f}")
    print(f"  std   : {w.std():.6f}")
    print(f"  abs>0 : {(w.abs() > 1e-6).sum().item()} / {w.numel()}")
    print(f"  per-output-channel std: min={w.std(dim=1).min():.4f}  max={w.std(dim=1).max():.4f}")
else:
    print(f"  KEY NOT FOUND: {pfn_key}")
    print(f"  Available voxel_encoder keys: {[k for k in sd if 'voxel' in k]}")

# ── 3. Trace raw PFN internals on synthetic input ────────────────────────────
print("\n=== 3. PFN internal trace (synthetic KITTI-like input) ===")
np.random.seed(0)
N = 20000
x = np.random.uniform(5.0, 50.0,  N).astype(np.float32)
y = np.random.uniform(-20.0, 20.0, N).astype(np.float32)
z = np.random.uniform(-1.9, 0.4,  N).astype(np.float32)
r = np.random.uniform(0.05, 0.5,  N).astype(np.float32)
synth = np.stack([x, y, z, r], axis=1)

model = build_pointpillars(painted=False)
ok = model.load_pretrained(args.checkpoint)
model.eval()

with torch.no_grad():
    pnp, cnp, nnp = model._voxelise(synth, model.cfg.max_voxels_test)
    print(f"  Pillars: {pnp.shape[0]}   pts shape: {pnp.shape}")

    # Manually replicate PFN forward to inspect pre-BN and post-BN values
    pfn = model.pfn
    pillars_t = torch.from_numpy(pnp)
    num_pts_t = torch.from_numpy(nnp)
    coords_t  = torch.from_numpy(cnp)
    P, Npts, C = pillars_t.shape
    mask_f = (torch.arange(Npts).unsqueeze(0) < num_pts_t.unsqueeze(1)).unsqueeze(-1).float()

    # Cluster mean offsets
    centre = (pillars_t[:,:,:3] * mask_f).sum(1) / num_pts_t.float().clamp(1).unsqueeze(1)
    d_cluster = pillars_t[:,:,:3] - centre.unsqueeze(1)

    # Voxel centre offsets
    ix = coords_t[:,3].float(); iy = coords_t[:,2].float()
    xp = (ix*0.16 + 0.08).view(P,1,1).expand(P,Npts,1)
    yp = (iy*0.16 - 39.68 + 0.08).view(P,1,1).expand(P,Npts,1)
    zp = pillars_t.new_full((P,Npts,1), -1.0)
    augmented = torch.cat([pillars_t[:,:,:4], d_cluster,
                           pillars_t[:,:,0:1]-xp, pillars_t[:,:,1:2]-yp,
                           pillars_t[:,:,2:3]-zp], dim=-1) * mask_f

    print(f"  Augmented tensor — mean={augmented[mask_f.squeeze(-1).bool()].mean():.4f}  "
          f"std={augmented[mask_f.squeeze(-1).bool()].std():.4f}  "
          f"min={augmented[mask_f.squeeze(-1).bool()].min():.4f}  "
          f"max={augmented[mask_f.squeeze(-1).bool()].max():.4f}")

    # Per-channel stats of augmented tensor (valid points only)
    valid = mask_f.squeeze(-1).bool()   # (P, N)
    aug_valid = augmented[valid]        # (n_valid_pts, 10)
    for ch in range(10):
        ch_vals = aug_valid[:, ch]
        print(f"    ch{ch:02d} mean={ch_vals.mean():+8.4f}  std={ch_vals.std():7.4f}  "
              f"min={ch_vals.min():+8.4f}  max={ch_vals.max():+8.4f}")

    flat = augmented.view(P * Npts, -1)
    pre_bn = pfn.layers[0](flat)   # linear output before BN
    print(f"  Pre-BN linear — mean={pre_bn.mean():.4f}  std={pre_bn.std():.4f}  "
          f"min={pre_bn.min():.4f}  max={pre_bn.max():.4f}")

    bn = pfn.layers[1]
    post_bn = bn(pre_bn)
    print(f"  Post-BN       — mean={post_bn.mean():.4f}  std={post_bn.std():.4f}  "
          f"min={post_bn.min():.4f}  max={post_bn.max():.4f}")

    pf = model.pfn(pillars_t, num_pts_t, coords_t)
    print(f"  PFN output    — mean={pf.mean():.4f}  std={pf.std():.4f}  "
          f"min={pf.min():.4f}  max={pf.max():.4f}")

# ── 4. PFN BN per-channel stats ───────────────────────────────────────────────
print("\n=== 4. PFN BatchNorm per-channel stats ===")
bn = model.pfn.layers[1]
print(f"  {'ch':>3}  {'run_mean':>10}  {'run_var':>10}  {'gamma':>8}  {'beta':>8}  {'scale=gamma/sqrt(var)':>22}")
for i in range(64):
    scale = bn.weight[i].item() / (bn.running_var[i].item() + 1e-3) ** 0.5
    flag = '  ← EXPLODES' if abs(scale) > 10 else ''
    print(f"  {i:>3}  {bn.running_mean[i].item():>10.4f}  {bn.running_var[i].item():>10.6f}"
          f"  {bn.weight[i].item():>8.4f}  {bn.bias[i].item():>8.4f}  {scale:>22.4f}{flag}")

# ── 5. Trace activations through backbone block by block ─────────────────────
print("\n=== 5. Activation trace through backbone + neck ===")
with torch.no_grad():
    pnp2, cnp2, nnp2 = model._voxelise(synth, model.cfg.max_voxels_test)
    pf2  = model.pfn(torch.from_numpy(pnp2), torch.from_numpy(nnp2), torch.from_numpy(cnp2))
    bev2 = model.scatter(pf2, torch.from_numpy(cnp2), batch_size=1)
    print(f"  BEV  — mean={bev2.mean():.4f}  std={bev2.std():.4f}  max={bev2.abs().max():.4f}")

    x = bev2
    for bi, block in enumerate(model.backbone.blocks):
        x = block(x)
        print(f"  Backbone block {bi} out — mean={x.mean():.4f}  std={x.std():.4f}  "
              f"max={x.abs().max():.4f}  shape={tuple(x.shape)}")

    neck_out = model.neck([b_out for b_out in
                           [model.backbone.blocks[i](
                               model.backbone.blocks[i-1](bev2) if i > 0 else bev2
                           ) for i in range(3)]])

    # Redo properly
    feats2 = model.backbone(bev2)
    for i, f in enumerate(feats2):
        print(f"  Backbone feat[{i}] — mean={f.mean():.4f}  std={f.std():.4f}  max={f.abs().max():.4f}")
    neck2 = model.neck(feats2)
    print(f"  Neck out — mean={neck2.mean():.4f}  std={neck2.std():.4f}  max={neck2.abs().max():.4f}")
    cls2, reg2, dir2 = model.head(neck2)
    print(f"  Head cls raw — min={cls2.min():.4f}  max={cls2.max():.4f}  mean={cls2.mean():.4f}")
    print(f"  Head cls sigmoid — max={torch.sigmoid(cls2).max():.4f}")

# ── 6. Check backbone BN running stats ───────────────────────────────────────
print("\n=== 6. Backbone first BN running stats ===")
bb_bn = model.backbone.blocks[0][1]
print(f"  running_mean — min={bb_bn.running_mean.min():.4f}  max={bb_bn.running_mean.max():.4f}")
print(f"  running_var  — min={bb_bn.running_var.min():.4f}   max={bb_bn.running_var.max():.4f}")

# ── 7. Check if BEV spatial stride matches backbone expectation ───────────────
print("\n=== 7. BEV → backbone spatial resolution check ===")
print(f"  BEV grid: {model.cfg.ny} × {model.cfg.nx}  = {model.cfg.ny}×{model.cfg.nx}")
print(f"  Backbone strides: {model.cfg.backbone_strides}  → cumulative: "
      f"{model.cfg.backbone_strides[0]}, "
      f"{model.cfg.backbone_strides[0]*model.cfg.backbone_strides[1]}, "
      f"{model.cfg.backbone_strides[0]*model.cfg.backbone_strides[1]*model.cfg.backbone_strides[2]}")
expected_h = [model.cfg.ny // (2**i) for i in range(1, 4)]
expected_w = [model.cfg.nx // (2**i) for i in range(1, 4)]
print(f"  Expected feat map sizes: {list(zip(expected_h, expected_w))}")

# ── 8. Verify head cls bias (should be large negative for low base rate) ─────
print("\n=== 8. Head classification bias ===")
cls_bias = model.head.cls_out.bias
print(f"  cls_out bias — min={cls_bias.min():.4f}  max={cls_bias.max():.4f}  "
      f"mean={cls_bias.mean():.4f}")
print(f"  sigmoid(bias) — min={torch.sigmoid(cls_bias).min():.4f}  "
      f"max={torch.sigmoid(cls_bias).max():.4f}")
print(f"  (KITTI prior-init bias would be around -4.6 → sigmoid≈0.01)")
