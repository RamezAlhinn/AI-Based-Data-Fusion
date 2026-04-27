import argparse

import numpy as np
import torch
from modeling.deeplab import DeepLab
from PIL import Image
from torchvision import transforms
import matplotlib.pyplot as plt

PASCAL_COLORS = [
    [0, 0, 0], [128, 0, 0], [0, 128, 0], [128, 128, 0], [0, 0, 128],
    [128, 0, 128], [0, 128, 128], [128, 128, 128], [64, 0, 0],
    [192, 0, 0], [64, 128, 0], [192, 128, 0], [64, 0, 128],
    [192, 0, 128], [64, 128, 128], [192, 128, 128], [0, 64, 0],
    [128, 64, 0], [0, 192, 0], [128, 192, 0], [0, 64, 128]
]

_preprocess = transforms.Compose([
    transforms.Resize((513, 513)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])


def load_model(checkpoint_path: str):
    model = DeepLab(
        backbone='resnet',
        output_stride=16,
        num_classes=21,
        sync_bn=False,
        freeze_bn=False
    )
    checkpoint = torch.load(checkpoint_path, map_location=torch.device('cpu'), weights_only=False)
    model.load_state_dict(checkpoint['state_dict'])
    model.eval()
    if torch.cuda.is_available():
        model = model.to('cuda')
    return model


def segment_image(model, image: Image.Image) -> np.ndarray:
    """Run DeepLab on a PIL image, return a (H, W) class-index array."""
    input_tensor = _preprocess(image).unsqueeze(0)
    if torch.cuda.is_available():
        input_tensor = input_tensor.to('cuda')
    with torch.no_grad():
        output = model(input_tensor)
    return np.argmax(output.data.cpu().numpy(), axis=1)[0]


def decode_segmap(label_mask: np.ndarray) -> np.ndarray:
    """Convert a (H, W) class-index array to an (H, W, 3) RGB image."""
    r = np.zeros_like(label_mask, dtype=np.uint8)
    g = np.zeros_like(label_mask, dtype=np.uint8)
    b = np.zeros_like(label_mask, dtype=np.uint8)
    for cls, color in enumerate(PASCAL_COLORS):
        idx = label_mask == cls
        r[idx], g[idx], b[idx] = color
    return np.stack([r, g, b], axis=2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run DeepLab segmentation on an image')
    parser.add_argument('--checkpoint', required=True, help='Path to deeplab-resnet.pth.tar')
    parser.add_argument('--image', required=True, help='Path to input image')
    args = parser.parse_args()

    model = load_model(args.checkpoint)
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
    plt.show()
