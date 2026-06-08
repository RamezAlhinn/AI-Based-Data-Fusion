"""
YOLO instance segmentation wrapper for PointPainting.

Provides two public functions used by the painting node and the isolation test:
    load_model(checkpoint_path)      — load YOLO11m-seg (auto-downloads if not found)
    segment_image(model, image)      — run segmentation, return (H, W) class ID array

Design decisions:
    - YOLO11m-seg is used — the medium model (~42 MB) rather than nano/small.
      The extra capacity is needed for dark/underexposed objects like the black BMW
      in this dataset, and improves person detection at distance.
    - Label mask is initialised to -1 (not 0) because COCO class 0 = person.
      Zero-initialisation would label all background pixels as person.
    - Confidence threshold is set to 0.25 (YOLO's validated default). 0.5 caused
      missed detections on dark vehicles; 0.15 caused false bus/truck labels.
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
    Load YOLO segmentation model. Auto-downloads if not found.
    Pass checkpoint_path to override with a local file.
    """
    from ultralytics import YOLO
    import os

    if checkpoint_path:
        model_path = checkpoint_path
    else:
        # Find model in workspace or default to yolo11n-seg.pt (fastest)
        possible_paths = [
            '/workspace/models/yolo11n-seg.pt',
            '/workspace/models/yolo11m-seg.pt',
            '/workspace/yolo11n-seg.pt',
            '/workspace/ros2_ws/yolo11n-seg.pt',
            os.path.join(os.path.dirname(__file__), '../../../../models/yolo11n-seg.pt'),
            os.path.join(os.path.dirname(__file__), '../../../../../../models/yolo11n-seg.pt'),
            os.path.join(os.path.dirname(__file__), '../../../../yolo11n-seg.pt'),
            os.path.join(os.path.dirname(__file__), '../../../../../../yolo11n-seg.pt'),
            '/workspace/yolo11m-seg.pt',
            '/workspace/ros2_ws/yolo11m-seg.pt',
            'yolo11n-seg.pt'
        ]
        model_path = 'yolo11n-seg.pt'
        for p in possible_paths:
            if os.path.exists(p):
                model_path = p
                break

    print(f"Loading YOLO segmentation model from: {os.path.abspath(model_path)}")
    return YOLO(model_path)


def segment_image(model, image: Image.Image) -> np.ndarray:
    """
    Run YOLO instance segmentation on a PIL image.
    Returns a (H, W) array with native YOLO/COCO class IDs per pixel, -1 = background.

    COCO class IDs (relevant for driving):
      0=person, 1=bicycle, 2=car, 3=motorcycle, 5=bus, 7=truck
    """
    img_np = np.array(image)
    h, w = img_np.shape[:2]

    # Prevent letterboxing by passing imgsz=(h, w)
    results = model(img_np, verbose=False, conf=0.25, imgsz=(h, w))
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
            # Use bilinear interpolation for cleaner boundaries
            mask_resized = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_LINEAR)
            # Tighter threshold (200) to prevent blur bleed
            label_mask[mask_resized > 200] = cls_id

    # Apply dilation to close interior holes in masks
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    clean_mask = np.full_like(label_mask, -1)
    for cls_id in np.unique(label_mask[label_mask >= 0]):
        binary  = (label_mask == cls_id).astype(np.uint8)
        dilated = cv2.dilate(binary, kernel, iterations=1)
        clean_mask[dilated == 1] = cls_id
    label_mask = clean_mask

    return label_mask


if __name__ == '__main__':
    import argparse
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description='Run YOLO segmentation on an image')
    parser.add_argument('--image', required=True, help='Path to input image')
    parser.add_argument('--checkpoint', default=None, help='Path to YOLO model file (default: yolo11m-seg.pt)')
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
