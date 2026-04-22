import torch
from modeling.deeplab import DeepLab
from torchvision import transforms
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

model = DeepLab(
    backbone='resnet',
    output_stride=16,
    num_classes=21,
    sync_bn=False,
    freeze_bn=False
)

# Specify the path to your downloaded pre-trained model
#checkpoint = torch.load('C:\\Users\\usuario\\Desktop\\THI\\AI-Data fusion research\\imagenes\\repository\\deeplab-resnet.pth.tar')
#CHANGE THE PATH
checkpoint = torch.load("C:\\Users\\usuario\\Desktop\\THI\\AI-Data fusion research\\imagenes\\repository\\deeplab-resnet.pth.tar", map_location=torch.device('cpu'),weights_only=False)
model.load_state_dict(checkpoint['state_dict'])
 
# Set model to evaluation mode
model.eval()


# Define preprocessing transformations
preprocess = transforms.Compose([
    transforms.Resize((513, 513)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])
 
# Load and preprocess image right now it is just for an image.
#CHANGE THE PATH
image = Image.open('C:\\Users\\usuario\\Desktop\\THI\\AI-Data fusion research\\imagenes\\1_car.jpg').convert('RGB')
input_tensor = preprocess(image).unsqueeze(0)  # Add batch dimension


# Move to the same device as the model
if torch.cuda.is_available():
    input_tensor = input_tensor.to('cuda')
    model = model.to('cuda')
 
# Disable gradient calculation for inference
with torch.no_grad():
    output = model(input_tensor)
    
# The output is a tensor of shape [1, num_classes, height, width]
# Convert it to a segmentation map by taking the argmax along the class dimension
pred = output.data.cpu().numpy()
pred = np.argmax(pred, axis=1)

def decode_segmap(label_mask, dataset='pascal'):
    """
    Convert segmentation class indices to RGB colors
    """
    if dataset == 'pascal':
        n_classes = 21
        colors = [
            [0, 0, 0], [128, 0, 0], [0, 128, 0], [128, 128, 0], [0, 0, 128],
            [128, 0, 128], [0, 128, 128], [128, 128, 128], [64, 0, 0],
            [192, 0, 0], [64, 128, 0], [192, 128, 0], [64, 0, 128],
            [192, 0, 128], [64, 128, 128], [192, 128, 128], [0, 64, 0],
            [128, 64, 0], [0, 192, 0], [128, 192, 0], [0, 64, 128]
        ]
    # ... define colors for other datasets
    
    r = np.zeros_like(label_mask).astype(np.uint8)
    g = np.zeros_like(label_mask).astype(np.uint8)
    b = np.zeros_like(label_mask).astype(np.uint8)
    
    for l in range(0, n_classes):
        idx = label_mask == l
        r[idx] = colors[l][0]
        g[idx] = colors[l][1]
        b[idx] = colors[l][2]
    
    rgb = np.stack([r, g, b], axis=2)
    return rgb
 
# Convert prediction to color segmentation map
segmentation_map = decode_segmap(pred[0], dataset='pascal')


# Visualize
plt.figure(figsize=(10,5))

plt.subplot(1,2,1)
plt.imshow(image)
plt.title("Original")
plt.axis('off')

plt.subplot(1,2,2)
plt.imshow(segmentation_map)
plt.title("Segmentation")
plt.axis('off')

plt.show()