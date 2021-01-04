import torch.nn as nn

class DiceLoss(nn.Module):

    def __init__(self):
        super(DiceLoss, self).__init__()
        self.smooth = 1.0

    def forward(self, input, target):
        input  = input[:, 0].contiguous().view(-1)
        target = target[:, 0].contiguous().view(-1)
        inter = (input * target).sum()
        dsc = (2. * inter + self.smooth) / (
            input.sum() + target.sum() + self.smooth
        )
        return 1. - dsc