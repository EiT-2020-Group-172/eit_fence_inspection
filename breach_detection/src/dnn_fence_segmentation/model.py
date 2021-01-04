import torch
import torch.nn as nn

from model_parts import *

class UNet(nn.Module):
    def __init__(self, n_channels, n_classes, bilinear=True):
        super(UNet, self).__init__()
        self.n_channels = n_channels
        self.n_classes = n_classes
        self.bilinear = bilinear

        self.inc = DoubleConvolution(n_channels, 64)
        
        self.down1 = DownSample(64, 128)
        self.down2 = DownSample(128, 256)
        self.down3 = DownSample(256, 512)
        
        factor = 2 if bilinear else 1
        
        self.down4 = DownSample(512, 1024 // factor)
        
        self.up1 = UpSample(1024, 512 // factor, bilinear)
        self.up2 = UpSample(512, 256 // factor, bilinear)
        self.up3 = UpSample(256, 128 // factor, bilinear)
        self.up4 = UpSample(128, 64, bilinear)
        
        self.outc = OutConvolution(64, n_classes)

    def forward(self, x):
        x1 = self.inc(x)

        x2 = self.down1(x1)
        x3 = self.down2(x2)
        x4 = self.down3(x3)
        x5 = self.down4(x4)
        
        x = self.up1(x5, x4)
        x = self.up2(x, x3)
        x = self.up3(x, x2)
        x = self.up4(x, x1)
        
        logits = self.outc(x)

        return logits