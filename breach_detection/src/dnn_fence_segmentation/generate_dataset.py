import os
import cv2

from PIL import Image
from tqdm import tqdm

def resizeImage(directory):
    # Load image
    img = Image.open(directory)

    # Resize image
    size = (128, 128)
    img_resized = img.resize(size)

    return img_resized    

# Save a list of all filenames in a folder
filePath  = 'De-fencing/dataset/TrainingSet/'
#filePath   = 'De-fencing/dataset/TestSet/'
fileNames = os.listdir(filePath+'Training_Images/')

# Directory of where to save the images
pathImages = 'data/train/images/'
pathMasks  = 'data/train/labels/'

for fileName in tqdm(fileNames):
    fileName, fileFormat = fileName.split('.')
    images = resizeImage(filePath + 'Training_Images/' + fileName + '.jpg')
    masks  = resizeImage(filePath + 'Training_Labels/' + fileName + '.png')

    if not os.path.exists(pathImages):
        os.makedirs(pathImages)

    if not os.path.exists(pathMasks):
        os.makedirs(pathMasks)  
  
    images.save(pathImages+str(fileName.split('.')[0])+'.png')
    masks.save(pathMasks+str(fileName.split('.')[0])+'.png')
