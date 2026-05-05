import numpy as np
from PIL import Image

# COCO class IDs relevant for driving scenes
# YOLO uses the same COCO class IDs as DeepLabV3
# 0=person, 1=bicycle, 2=car, 3=motorcycle, 5=bus, 7=truck
# Note: YOLO COCO IDs differ from DeepLab — remapped below to our colour map IDs
YOLO_TO_PAINTING_CLASS = {
    0:  15,  # person     → painting class 15 (red)
    1:  2,   # bicycle    → painting class 2  (blue)
    2:  3,   # car        → painting class 3  (green)
    3:  4,   # motorcycle → painting class 4  (orange)
    5:  6,   # bus        → painting class 6  (yellow)
    7:  3,   # truck      → painting class 3  (green, same as car)
}

_model = None


def load_model(checkpoint_path: str = None):
    """
    Load YOLOv11n-seg. Downloads automatically on first use (~6 MB, cached).
    checkpoint_path is accepted for API compatibility but ignored.
    """
    global _model
    from ultralytics import YOLO
    _model = YOLO('yolo11n-seg.pt')
    return _model


def segment_image(model, image: Image.Image) -> np.ndarray:
    """
    Run YOLO instance segmentation on a PIL image.
    Returns a (H, W) array where each pixel holds the painting class ID
    (matched to our colour map) or 0 for background.
    """
    img_np = np.array(image)
    h, w = img_np.shape[:2]

    results = model(img_np, verbose=False)
    label_mask = np.zeros((h, w), dtype=np.int32)

    for result in results:
        if result.masks is None:
            continue

        masks = result.masks.data.cpu().numpy()   # (N, H', W')
        classes = result.boxes.cls.cpu().numpy().astype(int)  # (N,)

        for mask, yolo_cls in zip(masks, classes):
            painting_cls = YOLO_TO_PAINTING_CLASS.get(yolo_cls)
            if painting_cls is None:
                continue

            import cv2
            mask_u8 = (mask * 255).astype(np.uint8)
            mask_resized = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_NEAREST)
            label_mask[mask_resized > 127] = painting_cls

    return label_mask


if __name__ == '__main__':
    import argparse
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description='Run YOLO segmentation on an image')
    parser.add_argument('--image', required=True, help='Path to input image')
    parser.add_argument('--checkpoint', default=None, help='Ignored — YOLO auto-downloads')
    args = parser.parse_args()

    model = load_model()
    image = Image.open(args.image).convert('RGB')
    label_mask = segment_image(model, image)

    # Simple colour visualisation for verification
    COLORS = {2: (0, 0, 255), 3: (0, 255, 0), 4: (255, 128, 0),
              6: (255, 255, 0), 15: (255, 0, 0)}
    vis = np.array(image).copy()
    for cls_id, (r, g, b) in COLORS.items():
        vis[label_mask == cls_id] = (r, g, b)

    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1); plt.imshow(image); plt.title('Original'); plt.axis('off')
    plt.subplot(1, 2, 2); plt.imshow(vis); plt.title('YOLO Segmentation'); plt.axis('off')
    out = args.image.rsplit('.', 1)[0] + '_yolo_seg.png'
    plt.savefig(out)
    print(f'Saved: {out}')
