"""
Download the pretrained PointPillars KITTI 3-class checkpoint from the
mmdetection3d model zoo and save it to the workspace.

Run inside the container:
  python3 /workspace/AI-Based-Data-Fusion/download_pointpillars.py
"""

import os
import urllib.request

CHECKPOINT_URL = (
    "https://download.openmmlab.com/mmdetection3d/v1.0.0_models/pointpillars/"
    "hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class/"
    "hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth"
)

SAVE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "models",
    "pointpillars_kitti_3class.pth"
)


def _progress(block_num, block_size, total_size):
    downloaded = block_num * block_size
    if total_size > 0:
        pct = min(100.0, downloaded * 100.0 / total_size)
        mb  = downloaded / 1e6
        tot = total_size / 1e6
        print(f'\r  {pct:5.1f}%  {mb:.1f} / {tot:.1f} MB', end='', flush=True)


if __name__ == '__main__':
    if os.path.isfile(SAVE_PATH):
        size_mb = os.path.getsize(SAVE_PATH) / 1e6
        print(f'Checkpoint already exists ({size_mb:.1f} MB): {SAVE_PATH}')
    else:
        print(f'Downloading PointPillars KITTI 3-class checkpoint...')
        print(f'  URL : {CHECKPOINT_URL}')
        print(f'  Dest: {SAVE_PATH}')
        urllib.request.urlretrieve(CHECKPOINT_URL, SAVE_PATH, _progress)
        print(f'\nDone.  Saved to: {SAVE_PATH}')

    print('\nRun the pipeline with:')
    print(f'  python3 /workspace/test_pipeline_isolation.py '
          f'--seed 7 --pp-checkpoint {SAVE_PATH}')
