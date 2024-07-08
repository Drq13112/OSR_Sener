import numpy as np
from skimage import img_as_ubyte
from skimage.color import rgb2gray
import cv2
import torch

class CNN_NET(torch.nn.Module):
    def __init__(self):
        super(CNN_NET, self).__init__()
        self.cnn_layers = torch.nn.Sequential(
            torch.nn.Conv2d(in_channels=1, out_channels=8, kernel_size=3, padding=1, stride=1),
            torch.nn.MaxPool2d(kernel_size=2, stride=2),
            torch.nn.ReLU(),
            torch.nn.Conv2d(in_channels=8, out_channels=32, kernel_size=3, padding=1, stride=1),
            torch.nn.MaxPool2d(kernel_size=2, stride=2),
            torch.nn.ReLU(),
        )
        self.fc_layers = torch.nn.Sequential(
            torch.nn.Linear(7 * 7 * 32, 200),
            torch.nn.ReLU(),
            torch.nn.Linear(200, 100),
            torch.nn.ReLU(),
            torch.nn.Linear(100, 10),
            torch.nn.LogSoftmax(dim=1)
        )

    def forward(self, x):
        out = self.cnn_layers(x)
        out = out.view(-1, 7 * 7 * 32)
        out = self.fc_layers(out)
        return out

# Load the model
path = "weights.h5"
model = torch.load(path)

def ImagePreProcess(im_orig):
    im_gray = rgb2gray(im_orig)
    img_gray_u8 = img_as_ubyte(im_gray)
    
    (thresh, im_bw) = cv2.threshold(img_gray_u8, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    img_resized = cv2.resize(im_bw, (28, 28))
    
    im_gray_invert = 255 - img_resized
    im_final = im_gray_invert.reshape(1, 1, 28, 28)
    im_final = torch.from_numpy(im_final)
    im_final = im_final.type('torch.FloatTensor')
    
    ans = model(im_final)
    
    ans = ans[0].tolist().index(max(ans[0].tolist()))
    print("Predicted digit:", ans)

def main():
    try:
        # Read image from file
        image_path = "path_to_your_image22.jpg"
        frame = cv2.imread(image_path)
        ImagePreProcess(frame)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
