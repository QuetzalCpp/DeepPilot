from scipy.misc import imread, imresize
from keras.layers import Input, Dense, Convolution2D, Conv2D
from keras.layers import MaxPooling2D, AveragePooling2D
from keras.layers import ZeroPadding2D, Dropout, Flatten
from keras.layers import merge, Reshape, Activation, BatchNormalization
from keras.layers import concatenate
from keras.utils.conv_utils import convert_kernel
from keras import backend as K
from keras.models import Model
import tensorflow as tf
import numpy as np
import h5py
import math

from keras.utils.vis_utils import plot_model

def euc_lossRoll(y_true, y_pred):
    lx = K.sqrt(K.sum(K.square(y_true[:,:] - y_pred[:,:]), axis=1, keepdims=True))
    return (0.3 * lx)

def euc_lossPitch(y_true, y_pred):
    lx = K.sqrt(K.sum(K.square(y_true[:,:] - y_pred[:,:]), axis=1, keepdims=True))
    return (0.3 * lx)

def euc_lossYaw(y_true, y_pred):
    lx = K.sqrt(K.sum(K.square(y_true[:,:] - y_pred[:,:]), axis=1, keepdims=True))
    return (0.3 * lx)
    
def euc_lossAlt(y_true, y_pred):
    lx = K.sqrt(K.sum(K.square(y_true[:,:] - y_pred[:,:]), axis=1, keepdims=True))
    return (0.3 * lx)

def create_posenet_3_separated_outs(weights_path=None, tune=False):
    with tf.device('/gpu:0'):
        input = Input(shape=(224, 224, 3))
        
        conv1 = Conv2D(64,(7,7),strides=(2,2), padding='same',activation='relu',name='conv1')(input)
        pool1 = MaxPooling2D(pool_size=(3,3),strides=(2,2), padding='same',name='pool1')(conv1)
        norm1 = BatchNormalization(axis=3, name='norm1')(pool1)
        reduction2 = Conv2D(64,(1,1), padding='same',activation='relu',name='reduction2')(norm1)
        conv2 = Conv2D(192,(3,3),padding='same',activation='relu',name='conv2')(reduction2)
        norm2 = BatchNormalization(axis=3, name='norm2')(conv2)
        pool2 = MaxPooling2D(pool_size=(3,3),strides=(2,2),padding='valid',name='pool2')(norm2)
        icp1_reduction1 = Conv2D(96,(1,1),padding='same',activation='relu',name='icp1_reduction1')(pool2)
        icp1_out1 = Conv2D(128,(3,3),padding='same',activation='relu',name='icp1_out1')(icp1_reduction1)
        icp1_reduction2 = Conv2D(16,(1,1),padding='same',activation='relu',name='icp1_reduction2')(pool2)
        icp1_out2 = Conv2D(32,(5,5),padding='same',activation='relu',name='icp1_out2')(icp1_reduction2)
        icp1_pool = MaxPooling2D(pool_size=(3,3),strides=(1,1),padding='same',name='icp1_pool')(pool2)
        icp1_out3 = Conv2D(32,(1,1),padding='same',activation='relu',name='icp1_out3')(icp1_pool)
        icp1_out0 = Conv2D(64,(1,1),padding='same',activation='relu',name='icp1_out0')(pool2)
        icp2_in = concatenate([icp1_out0, icp1_out1, icp1_out2, icp1_out3], axis=-1)

        icp2_reduction1 = Conv2D(128,(1,1),padding='same',activation='relu',name='icp2_reduction1')(icp2_in)
        icp2_out1 = Conv2D(192,(3,3),padding='same',activation='relu',name='icp2_out1')(icp2_reduction1)
        icp2_reduction2 = Conv2D(32,(1,1),padding='same',activation='relu',name='icp2_reduction2')(icp2_in)
        icp2_out2 = Conv2D(96,(5,5),padding='same',activation='relu',name='icp2_out2')(icp2_reduction2)
        icp2_pool = MaxPooling2D(pool_size=(3,3),strides=(1,1),padding='same',name='icp2_pool')(icp2_in)
        icp2_out3 = Conv2D(64,(1,1),padding='same',activation='relu',name='icp2_out3')(icp2_pool)
        icp2_out0 = Conv2D(128,(1,1),padding='same',activation='relu',name='icp2_out0')(icp2_in)
        icp2_out = concatenate([icp2_out0, icp2_out1, icp2_out2, icp2_out3], axis=-1)

        icp3_in = MaxPooling2D(pool_size=(3,3),strides=(2,2),padding='same',name='icp3_in')(icp2_out)
        icp3_reduction1 = Conv2D(96,(1,1),padding='same',activation='relu',name='icp3_reduction1')(icp3_in)
        icp3_out1 = Conv2D(208,(3,3),padding='same',activation='relu',name='icp3_out1')(icp3_reduction1)
        icp3_reduction2 = Conv2D(16,(1,1),padding='same',activation='relu',name='icp3_reduction2')(icp3_in)
        icp3_out2 = Conv2D(48,(5,5),padding='same',activation='relu',name='icp3_out2')(icp3_reduction2)
        icp3_pool = MaxPooling2D(pool_size=(3,3),strides=(1,1),padding='same',name='icp3_pool')(icp3_in)
        icp3_out3 = Conv2D(64,(1,1),padding='same',activation='relu',name='icp3_out3')(icp3_pool)
        icp3_out0 = Conv2D(192,(1,1),padding='same',activation='relu',name='icp3_out0')(icp3_in)
        icp3_out = concatenate([icp3_out0, icp3_out1, icp3_out2, icp3_out3], axis = -1)

        cls1_pool = AveragePooling2D(pool_size=(5,5),strides=(3,3),padding='valid',name='cls1_pool')(icp3_out)
        cls1_reduction_speed = Conv2D(128,(1,1),padding='same',activation='relu',name='cls1_reduction_speed')(cls1_pool)
        cls1_fc1_flat = Flatten()(cls1_reduction_speed)    

        # Dedicate dense layer for each axis
        cls1_branch1_speed = Dense(1024,activation='relu',name='cls1_branch1_speed')(cls1_fc1_flat)
        cls1_branch2_speed = Dense(1024,activation='relu',name='cls1_branch2_speed')(cls1_fc1_flat)
        cls1_branch3_speed = Dense(1024,activation='relu',name='cls1_branch3_speed')(cls1_fc1_flat)
        cls1_branch4_speed = Dense(1024,activation='relu',name='cls1_branch4_speed')(cls1_fc1_flat)

        # Layer for each axis (1 neuron layer)
        roll = Dense(1,name='roll')(cls1_branch1_speed)
        pitch = Dense(1,name='pitch')(cls1_branch2_speed)
        yaw = Dense(1,name='yaw')(cls1_branch3_speed)
        altitude = Dense(1,name='altitude')(cls1_branch4_speed)
        
        posenet = Model(inputs=input, outputs=[roll, pitch, yaw, altitude])

    if tune:
        if weights_path:
            weights_data = np.load(weights_path, allow_pickle = True).item()
            for layer in posenet.layers:
                if layer.name in weights_data.keys():
                    layer_weights = weights_data[layer.name]
                    layer.set_weights((layer_weights['weights'], layer_weights['biases']))
            print("FINISHED SETTING THE WEIGHTS!")
    return posenet
