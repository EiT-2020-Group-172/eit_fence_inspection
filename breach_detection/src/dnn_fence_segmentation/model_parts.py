import torch
import torch.nn as nn
import torch.nn.functional as F

class DoubleConvolution(nn.Module):
    def __init__(self, feat_in, feat_out, feat_mid=None):
        super().__init__()
        if not feat_mid:
            feat_mid = feat_out
        self.doubleConv = nn.Sequential(
            nn.Conv2d(feat_in, feat_mid, kernel_size=3, padding=1),
            nn.BatchNorm2d(feat_mid),
            nn.ReLU(inplace=True),
            nn.Conv2d(feat_mid, feat_out, kernel_size=3, padding=1),
            nn.BatchNorm2d(feat_out),
            nn.ReLU(inplace=True),
        )    
    
    def forward(self, x):
        return self.doubleConv(x)

class DownSample(nn.Module):
    def __init__(self, feat_in, feat_out):
        super().__init__()
        self.maxpoolConv = nn.Sequential(
            nn.MaxPool2d(2),
            DoubleConvolution(feat_in, feat_out),
        )

    def forward(self, x):
        return self.maxpoolConv(x)

class UpSample(nn.Module):
    def __init__(self, feat_in, feat_out, bilinear=True):
        super().__init__()
        if bilinear:
            self.up   = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
            self.conv = DoubleConvolution(feat_in, feat_out, feat_in // 2) 
        else:
            self.up   = nn.ConvTranspose2d(feat_in, feat_in // 2, kernel_size=2, stride=2)
            self.conv = DoubleConvolution(feat_in, feat_out)

    def forward(self, x1, x2):
        x1 = self.up(x1)

        diffY = x2.size()[2] - x1.size()[2]
        diffX = x2.size()[3] - x1.size()[3]

        x1 = F.pad(x1, [diffX // 2, diffX - diffX // 2,
                        diffY // 2, diffY - diffY // 2])

        x = torch.cat([x2, x1], dim=1)

        return self.conv(x)

class OutConvolution(nn.Module):
    def __init__(self, feat_in, feat_out):
        super(OutConvolution, self).__init__()
        self.conv = nn.Conv2d(feat_in, feat_out, kernel_size=1)

    def forward(self, x):
        return self.conv(x)
