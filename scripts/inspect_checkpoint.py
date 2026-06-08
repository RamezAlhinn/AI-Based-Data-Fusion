"""
Print all key names and tensor shapes in a checkpoint.
Run inside the container:
  python3 /workspace/AI-Based-Data-Fusion/inspect_checkpoint.py \
      /workspace/AI-Based-Data-Fusion/pointpillars_kitti_3class.pth
"""
import sys
import torch

path = sys.argv[1] if len(sys.argv) > 1 else \
       '/workspace/models/pointpillars_kitti_3class.pth'

ckpt = torch.load(path, map_location='cpu', weights_only=False)
sd   = ckpt.get('state_dict', ckpt) if isinstance(ckpt, dict) else ckpt

print(f'Top-level keys: {list(ckpt.keys()) if isinstance(ckpt, dict) else "(raw state_dict)"}')
print(f'\nTotal tensors: {len(sd)}\n')
for k, v in sd.items():
    print(f'  {k:<70s} {str(tuple(v.shape)):<25s} {v.dtype}')
