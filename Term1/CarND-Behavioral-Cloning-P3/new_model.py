import numpy as np 
from keras.models import Model, Sequential
from keras.layers import Dense, Activation, Convolution2D, Cropping2D
from keras.layers import Flatten, Lambda
import os
import csv
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
import cv2
import sklearn

#image parameters
in_height = 160
in_width = 320
in_channels = 3

scaling_factor = 2.0 #used to resize the input image

resized_height = int(in_height/scaling_factor)
resized_width = int(in_width/scaling_factor)

# steering bias added to the main steering values for right and left cameras images
right_bias = -0.15
left_bias = 0.15
data_path = 'new_data/'
print('***LOADING THE IMAGES FROM THE SIMULATOR***')
samples = []
with open(data_path +'driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)
    for line in reader:
        samples.append(line)
print(samples[0])
# split the array containing the info into 2 sets

train_samples, validation_samples = train_test_split(samples, test_size=0.3)
total_images = len(samples)
training_set_size = len(train_samples)
validation_set_size = len(validation_samples)

print('size of the total set: ', total_images)
print('size of the training set:', training_set_size)
print('size of the validation set:',validation_set_size)

#X_train = np.empty([total_images*2,resized_height,resized_width,3], dtype=np.float_)
#y_train = np.empty([total_images*2], dtype=np.float_)

def generator(samples, batch_size=32, center_image_only=False):
	num_samples = len(samples)
	while 1: # Loop forever so the generator never terminates
		shuffle(samples)
		for offset in range(0, num_samples, batch_size):
			batch_samples = samples[offset:offset+batch_size]
			batch_size = len(batch_samples)
			if center_image_only:
				X_train = np.empty([batch_size*2,in_height,in_width,3], dtype=np.float_)
				y_train = np.empty([batch_size*2], dtype=np.float_)
			else:
				X_train = np.empty([batch_size*4,in_height,in_width,3], dtype=np.float_)
				y_train = np.empty([batch_size*4], dtype=np.float_)
			for index, sample in enumerate(batch_samples):
				source_path = sample[0]
				filename = source_path.split('/')[-1]
				current_path = data_path+'IMG/'+ filename
				image = cv2.imread(current_path)
				#data augmentation with the flipped image
				flip_image = cv2.flip(image, 1)
				#resize the image
				#image = cv2.resize(image, None, fx=1/scaling_factor, fy=1/scaling_factor)
				#flip_image = cv2.resize(flip_image, None, fx=1/scaling_factor, fy=1/scaling_factor)
				#convert to YUV space
				yuv_image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
				yuv_flip_image = cv2.cvtColor(flip_image, cv2.COLOR_BGR2YUV)
				#save the image in a numpy array format
				X_train[index] = yuv_image
				X_train[index+batch_size] = yuv_flip_image
				y_train[index] = float(sample[3])
				y_train[index+batch_size] = -float(sample[3])
				if not center_image_only:
					# add the right and left images to the set
					# right
					right_image_path = sample[2]
					filename_right = right_image_path.split('/')[-1]
					current_path_right = data_path+'IMG/'+ filename_right
					right_image = cv2.imread(current_path_right)
					#right_image = cv2.resize(right_image, None, fx=1/scaling_factor, fy=1/scaling_factor)
					yuv_right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2YUV)
					# left
					left_image_path = sample[1]
					filename_left = left_image_path.split('/')[-1]
					current_path_left = data_path+'IMG/'+ filename_left
					left_image = cv2.imread(current_path_left)
					#left_image = cv2.resize(left_image, None, fx=1/scaling_factor, fy=1/scaling_factor)
					yuv_left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2YUV)
					X_train[index+batch_size*2] = yuv_right_image
					X_train[index+batch_size*3] = yuv_left_image
					y_train[index+batch_size*2] = float(sample[3])+right_bias
					y_train[index+batch_size*3] = float(sample[3])+left_bias
			yield sklearn.utils.shuffle(X_train, y_train)

img_input_shape = (in_height,in_width,3)
#kernel 1 parameters
k1_row = 5
k1_column = 5
stride1 = (2,2)
#kernel 2 parameters
k2_row = 3
k2_column = 3
stride2 = (1,1)

print('*** CREATE THE MODEL ***')
model = Sequential()
model.add(Lambda(lambda x: x/255.0 -0.5, input_shape = img_input_shape))
model.add(Cropping2D(cropping=((25*2,10*2), (0,0)), input_shape=img_input_shape))
model.add(Convolution2D(24, k1_row, k1_column,
            border_mode='valid',
            subsample=stride1,
			activation='relu'))
model.add(Convolution2D(36, k1_row, k1_column,
            border_mode='valid',
            subsample=stride1,
            activation = 'relu'))
model.add(Convolution2D(48, k1_row, k1_column,
            border_mode='valid',
            subsample=stride1,
            activation = 'relu'))
model.add(Convolution2D(64, k2_row, k2_column,
            border_mode='valid',
            subsample=stride2,
            activation = 'relu'))
model.add(Convolution2D(64, k2_row, k2_column,
            border_mode='valid',
            subsample=stride2,
            activation = 'relu'))

model.add(Flatten())
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dense(1))

print('***TRAIN***')

train_generator = generator(train_samples, batch_size=32, center_image_only=False)
validation_generator = generator(validation_samples, batch_size=32, center_image_only=True)

model.compile(loss='mse', optimizer = 'adam')
#model.fit(X_train,y_train, batch_size = 5, validation_split=0.2, shuffle = True, nb_epoch = 3)

model.fit_generator(train_generator,
					samples_per_epoch=training_set_size*4,
					validation_data=validation_generator, # *2 because of the data augmentation
            		nb_val_samples=validation_set_size*2,
            		nb_epoch=4)

print('***SAVE***')
model.save('model_new.h5')