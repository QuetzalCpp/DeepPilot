__author__	= "L. Oyuki Rojas-Perez"
from tqdm import tqdm
import numpy as np
import os.path
import sys
import random
import math
import cv2
import gc

directory = './dataset/ardrone_dataset/'
dataset_train = 'train/train.txt'
dataset_test = 'test/test.txt'

class datasource(object):
	def __init__(self, images, speed):
		self.images = images
		self.speed = speed

def preprocess(images):
	images_out = [] #final result
	#Resize input images
	for i in tqdm(range(len(images))):
		X = cv2.imread(images[i])
		X = cv2.resize(X, (224, 224))
		X = np.transpose(X,(2,0,1))
		X = np.squeeze(X)
		X = np.transpose(X, (1,2,0))
		Y = np.expand_dims(X, axis=0)
		images_out.append(Y)
	del X, i
	gc.collect()
	return images_out

def preprocess_2(image):
	images_out = [] #final result

	X = cv2.resize(image, (224, 224))
	N = 1
	mean = np.zeros((1, 3, 224, 224))
	X = np.transpose(X,(2,0,1))
	X = X - mean
	X = np.squeeze(X)
	X = np.transpose(X, (1,2,0))
	Y = np.expand_dims(X, axis=0)
	images_out.append(Y)
	return images_out

def get_data(dataset):
	speed = []
	images = []
	
	with open(directory+dataset) as f:
		next(f)  # skip the header line
		next(f)
		next(f)
		for line in f:
			fname,p0,p1,p2,p3 = line.split()
			p0 = float(p0)
			p1 = float(p1)
			p2 = float(p2)
			p3 = float(p3)
			speed.append((p0,p1,p2,p3))
			images.append(directory+fname)
	images_out = preprocess(images)
	return datasource(images_out, speed)

def getTrainSource():
	datasource_train = get_data(dataset_train)
	datasource_test = get_data(dataset_test)

	images_train = []
	speed_train = []

	images_test = []
	speed_test = []

	for i in range(len(datasource_train.images)):
		images_train.append(datasource_train.images[i])
		speed_train.append(datasource_train.speed[i])
	for i in range(len(datasource_test.images)):
		images_test.append(datasource_test.images[i])
		speed_test.append(datasource_test.speed[i])

	return datasource(images_train, speed_train), datasource(images_test, speed_test)

def getTestSource():
	datasource_test = get_data(dataset_test)
	
	images_test = []
	speed_test = []

	for i in range(len(datasource_test.images)):
		images_test.append(datasource_test.images[i])
		speed_test.append(datasource_test.speed[i])

	return datasource(images_test, speed_test)	
