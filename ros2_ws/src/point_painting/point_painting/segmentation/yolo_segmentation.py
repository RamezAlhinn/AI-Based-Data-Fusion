"""
YOLO instance segmentation wrapper for PointPainting.

Provides two public functions used by the painting node and the isolation test:
    load_model(checkpoint_path)      — load YOLO11n-seg (auto-downloads if not found)
    segment_image(model, image)      — run segmentation, return (H, W) class ID array

Design decisions:
    - YOLO11n-seg is used (not YOLO26) — YOLO11 is the stable production release
      with well-validated COCO accuracy; YOLO26 is experimental and less reliable.
    - Label mask is initialised to -1 (not 0) because COCO class 0 = person.
      Zero-initialisation would label all background pixels as person.
    - Confidence threshold is set to 0.5 (YOLO recommended minimum). Values below
      this cause false positives — e.g. a car being hallucinated as bus or truck.
    - Masks are resized back to the full input image resolution so pixel
      coordinates from the LiDAR projection map directly onto the mask.
    - Mask priority ordering: vehicles painted first, person/bicycle/motorcycle
      painted last so a pedestrian in front of a car is labelled as person.

Built on top of the initial YOLO script from carlosaterans-cmd (Code branch)
but with class labels preserved — their version published a binary mask and
discarded the class information.
"""

import numpy as np
import cv2
from PIL import Image


def load_model(checkpoint_path: str = None):
    """
    Load YOLO11n-seg model. Auto-downloads on first use (~6 MB, cached).
    Pass checkpoint_path to override with a local file.
    """
    from ultralytics import YOLO
    model_path = checkpoint_path if checkpoint_path else 'yolo11n-seg.pt'
    return YOLO(model_path)


def segment_image(model, image: Image.Image) -> np.ndarray:
    """
    Run YOLO instance segmentation on a PIL image.
    Returns a (H, W) array with native YOLO/COCO class IDs per pixel, 0 = background.

    COCO class IDs (relevant for driving):
      0=person, 1=bicycle, 2=car, 3=motorcycle, 5=bus, 7=truck
    """
    img_np = np.array(image)
    h, w = img_np.shape[:2]

    results = model(img_np, verbose=False, conf=0.5)
    label_mask = np.full((h, w), -1, dtype=np.int32)  # -1 = background/no detection

    # Priority order: vehicles first, vulnerable road users last so they
    # overwrite vehicle masks when a person stands in front of a car.
    PRIORITY = {0: 10, 1: 9, 3: 8}  # person > bicycle > motorcycle > everything else

    for result in results:
        if result.masks is None:
            continue

        masks = result.masks.data.cpu().numpy()        # (N, H', W')
        classes = result.boxes.cls.cpu().numpy().astype(int)  # (N,) — native COCO IDs

        pairs = sorted(zip(masks, classes), key=lambda mc: PRIORITY.get(mc[1], 0))
        for mask, cls_id in pairs:
            mask_u8 = (mask * 255).astype(np.uint8)
            mask_resized = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_NEAREST)
            label_mask[mask_resized > 127] = cls_id

    return label_mask


if __name__ == '__main__':
    import argparse
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description='Run YOLO segmentation on an image')
    parser.add_argument('--image', required=True, help='Path to input image')
    parser.add_argument('--checkpoint', default=None, help='Path to YOLO model file (default: yolo11n-seg.pt)')
    args = parser.parse_args()

    model = load_model(args.checkpoint)
    image = Image.open(args.image).convert('RGB')
    label_mask = segment_image(model, image)

    print('Detected classes:', np.unique(label_mask[label_mask > 0]))
    print('Class names:', {i: model.names[i] for i in np.unique(label_mask) if i in model.names})

    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1); plt.imshow(image); plt.title('Original'); plt.axis('off')
    plt.subplot(1, 2, 2); plt.imshow(label_mask, cmap='tab20'); plt.title('YOLO Classes'); plt.axis('off')
    out = args.image.rsplit('.', 1)[0] + '_yolo_seg.png'
    plt.savefig(out)
    print(f'Saved: {out}')
