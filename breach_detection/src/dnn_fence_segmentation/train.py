import os
import torch
import numpy as np
import torch.nn as nn
import matplotlib.pyplot as plt
import torch.nn.functional as F

from PIL import Image
from tqdm import tqdm
from model import UNet 
from torch import optim
from loss import DiceLoss
from torch.utils.data import TensorDataset, DataLoader, random_split

class FenceSegmentationNet():
    def __init__(self, filePathTrain):
        # Hyperparameters
        self.batchSize    =  1
        self.numEpochs    =  10
        self.learningRate =  0.001
        self.validPercent =  0.1
        self.trainShuffle = True
        self.testShuffle  = False
        self.momentum     = 0.99
        self.imageDim     = 128

        # Variables
        self.imageDirectory = filePathTrain
        self.labelDirectory = filePathTrain
        self.numChannels    = 3
        self.numClasses     = 1

        # Device configuration
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Load dataset
        self.trainLoader = self.getTrainingLoader()
        #self.testLoader  = self.getTestLoader()

        # Setup model
        self.model = UNet(n_channels = self.numChannels, n_classes = self.numClasses, bilinear=True).to(self.device)
        #self.optimizer = torch.optim.RMSprop(self.model.parameters(), lr=self.learningRate, weight_decay=self.weightDecay, momentum=self.momentum)
        #self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learningRate)
        self.optimizer = torch.optim.SGD(self.model.parameters(), lr=self.learningRate, momentum=self.momentum)
        #self.scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(self.optimizer, 'min' if self.numClasses > 1 else 'max', patience=2)
        self.criterion = DiceLoss()

    def loadImages(self, directory):
        fileNames = os.listdir(directory+'images/')

        images = []
        labels = []
        for fileName in tqdm(fileNames, desc='Loading data'):
            image = Image.open(directory+'images/'+fileName)
            label = Image.open(directory+'labels/'+fileName).convert('L')
            label = label.point(lambda x: 0 if x<128 else 255, '1')
            
            # Converting the data into the format [Channels Width Height]
            label = np.asarray(label).reshape(1, self.imageDim, self.imageDim)
            image = np.asarray(image).reshape(3, self.imageDim, self.imageDim)

            labels.append(label)
            images.append(image)

        return images, labels

    def getTrainingLoader(self):
        print('Get trainings loader:')
        data, labels = self.loadImages(self.imageDirectory)

        # Convert image data and labels to tensor format
        tensorData    = torch.Tensor(data)
        tensorLabels  = torch.Tensor(labels)

        # Convert the two  tensors into one tensor dataset
        tensorDataset = TensorDataset(tensorData, tensorLabels)

        # Convert the tensor dataset into a dataloader with format [Batch Channels Width Height]
        trainLoader = DataLoader(tensorDataset, batch_size=self.batchSize, shuffle=self.trainShuffle)

        return trainLoader
    
    def getTestLoader(self):
        print('Get test loader:')
        data, labels = self.loadImages(self.filePathTest)

        # Convert image data and labels to tensor format
        tensorData    = torch.Tensor(data)
        tensorLabels  = torch.Tensor(labels)

        # Convert the two  tensors into one tensor dataset
        tensorDataset = TensorDataset(tensorData, tensorLabels)

        # Convert the tensor dataset into a dataloader with format [Batch Channels Width Height]
        testLoader = DataLoader(tensorDataset, batch_size=self.batchSize, shuffle=self.testShuffle)

        return testLoader

    def trainModel(self, patience=3):
        print('Train Model:')
        self.model.train()
        losses = []
        n_total_steps = len(self.trainLoader)
        for epoch in range(self.numEpochs):
            n_correct, epoch_loss = 0, 0
            tmpStr = 'Epoch [{:>3}/{:>3}]'.format(epoch+1,self.numEpochs)
            for (samples, labels) in tqdm(self.trainLoader,desc=tmpStr):
                samples = samples.to(self.device, dtype=torch.float32)
                labels  = labels.to(self.device, dtype=torch.long)

                # Forward pass
                outputs = self.model(samples)
                loss = self.criterion(outputs, labels)
                epoch_loss += loss.item()
            
                # Backward and optimize
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
            
            tot_loss = epoch_loss / n_total_steps
            print(tot_loss)
            losses.append(tot_loss)

        return losses

    def testModel(self): 
        print('Testing Model:')
        self.model.eval()
        n_correct = 0
        n_samples = 0
        tmpStr = 'Testing'
        for i, (samples, labels) in enumerate(tqdm(self.testLoader,desc=tmpStr)):
            samples   = samples.to(self.device, dtype=torch.float32)
            labels    = labels.to(self.device, dtype=torch.float32)

            outputs = self.model(samples)
            _, predicted = torch.max(outputs, 1)
            n_samples += labels.size(0)
            pred = torch.sigmoid(outputs)
            pred = (pred > 0.2).float()
            n_correct += dice_coeff(pred, labels).item()
            acc = 100.0 * n_correct / n_samples
        return acc           

    def plot_history(self, train_losses):
        print('Plotting epoch history')
        plt.figure(figsize=(3, 3))
        plt.xlabel('epoch')
        plt.ylabel('loss')
        plt.plot(train_losses, label='train')
        plt.ylim(0, 1)
        plt.legend()
        plt.grid()
        plt.tight_layout()
        plt.savefig('plot.png')

    def saveModel(self, fileName):
        torch.save(self.model.state_dict(), fileName)

    def loadModel(self, fileName):
        self.model.load_state_dict(torch.load(fileName))

if __name__ == "__main__":
    net = FenceSegmentationNet('data/train/')
    train_losses = net.trainModel()
    net.plot_history(train_losses)
    net.saveModel('models/model.pth')

    #net.loadModel('models/model')
    #accuracy = net.testModel()
    
    #print(f'Test accuracy', accuracy)

    print('Complete')
