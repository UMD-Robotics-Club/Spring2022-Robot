import numpy as np
import cv2 as cv
import pytesseract
'''
img = cv.imread('digits.png')
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# split the image into 40x40 cells
cells = [np.hsplit(row,100) for row in np.vsplit(gray,50)]

# convert the images into a numpy array
x = np.array(cells)

# prepare the training and test data
train = x[:,:50].reshape(-1,400).astype(np.float32)
test = x[:,50:100].reshape(-1,400).astype(np.float32)

# create labels for train and test data
k = np.arange(10)
train_labels = np.repeat(k,250)[:,np.newaxis]
test_labels = train_labels.copy()

all_k = []

#initiate kNN, train it on the training data, then test it with the test data with k=5
knn = cv.ml.KNearest_create()
knn.train(train, cv.ml.ROW_SAMPLE, train_labels)
ret,result,neighbours,dist = knn.findNearest(test,k=1)

#check for the accuracy of classification
# compare the guessed labels with test_labels to see which guesses are wrong
matches = result==test_labels
correct = np.count_nonzero(matches)
accuracy = correct*100/result.size





# Save the data
np.savez('knn_data.npz', train=train, train_labels=train_labels)

# now load the data
with np.load('knn_data.npz') as data:
    print(data.files)
    train = data['train']
    train_labels = data['train_labels']
'''

class classifier:
    def __init__(self, training_file):
        with np.load(training_file) as data:
            print(data.files)
            self.train = data['train']
            self.train_labels = data['train_labels']
        return

    def convert_to_array(self, frame, bounding_box):
        # gray scale the image
        number = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # bounding box is a tuple: (x, y, w, h)
        x, y, w, h = bounding_box[0], bounding_box[1],bounding_box[2],bounding_box[3]
        # crops the image to within the bounding box
        number = number[y:y+h, x:x+w]
        # resizes the image to 40x40
        number = cv.resize(number, (40,40))
        x = np.array(number)
        x.astype(np.float32)
        #ret, result, neighbours, dist = self.train.findNearest(x, k=5)
        #print(result)
        return number