import argparse

import numpy as np
import torch
from PIL import Image
from torchvision import transforms
from torchvision.models.segmentation import deeplabv3_resnet101, DeepLabV3_ResNet101_Weights

# Cityscapes-compatible class palette (21 COCO classes that overlap with driving scenes)
# Classes: 0=background, 2=car, 7=train, 14=bird, 15=cat, etc.
# More useful for driving: person=15, car=7, bicycle=2, motorcycle=4, bus=6
COCO_COLORS = [
    [0, 0, 0], [128, 0, 0], [0, 128, 0], [128, 128, 0], [0, 0, 128],
    [128, 0, 128], [0, 128, 128], [128, 128, 128], [64, 0, 0],
    [192, 0, 0], [64, 128, 0], [192, 128, 0], [64, 0, 128],
    [192, 0, 128], [64, 128, 128], [192, 128, 128], [0, 64, 0],
    [128, 64, 0], [0, 192, 0], [128, 192, 0], [0, 64, 128]
]

# Relevant class IDs for PointPainting (COCO-trained DeepLabV3):
# 0=background, 2=bicycle, 4=motorcycle, 6=bus, 7=train, 13=bench,
# 15=person, 17=cat, 18=dog, 19=horse, 20=sheep
DRIVING_CLASSES = {2: 'bicycle', 4: 'motorcycle', 6: 'bus', 7: 'train', 15: 'person'}

_preprocess = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])


def load_model(checkpoint_path: str = None):
    """
    Load DeepLabV3-ResNet101 pretrained on COCO (includes person, bicycle, car, bus).
    checkpoint_path is accepted for API compatibility but ignored — weights are
    downloaded automatically from torchvision on first use (~330 MB, cached).
    """
    weights = DeepLabV3_ResNet101_Weights.DEFAULT
    model = deeplabv3_resnet101(weights=weights)
    model.eval()
    if torch.cuda.is_available():
        model = model.to('cuda')
    return model


def segment_image(model, image: Image.Image) -> np.ndarray:
    """Run DeepLabV3 on a PIL image, return a (H, W) class-index array."""
    input_tensor = _preprocess(image).unsqueeze(0)
    if torch.cuda.is_available():
        input_tensor = input_tensor.to('cuda')
    with torch.no_grad():
        output = model(input_tensor)['out']
    return np.argmax(output.data.cpu().numpy(), axis=1)[0]


def decode_segmap(label_mask: np.ndarray) -> np.ndarray:
    """Convert a (H, W) class-index array to an (H, W, 3) RGB image."""
    r = np.zeros_like(label_mask, dtype=np.uint8)
    g = np.zeros_like(label_mask, dtype=np.uint8)
    b = np.zeros_like(label_mask, dtype=np.uint8)
    for cls, color in enumerate(COCO_COLORS):
        if cls >= len(COCO_COLORS):
            break
        idx = label_mask == cls
        r[idx], g[idx], b[idx] = color
    return np.stack([r, g, b], axis=2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run DeepLabV3 segmentation on an image')
    parser.add_argument('--image', required=True, help='Path to input image')
    parser.add_argument('--checkpoint', default=None, help='Ignored — torchvision weights used')
    args = parser.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    model = load_model()
    image = Image.open(args.image).convert('RGB')
    label_mask = segment_image(model, image)
    segmentation_map = decode_segmap(label_mask)

    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(image)
    plt.title('Original')
    plt.axis('off')
    plt.subplot(1, 2, 2)
    plt.imshow(segmentation_map)
    plt.title('Segmentation')
    plt.axis('off')
    out = args.image.rsplit('.', 1)[0] + '_segmentation.png'
    plt.savefig(out)
    print(f'Saved: {out}')
