import network_libs.helper_net as helper_net
import network_libs.net as net
import numpy as np
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint, EarlyStopping
import matplotlib.pyplot as plt
from keras import backend as K 

if __name__ == "__main__":
    K.clear_session()
    # Variables
    #change batch_size in function of your VRAM capacity, or for the sample size you want
    batch_size = 32
   
    # Train model
    # Model creation for separated outputs
    model = net.create_posenet_3_separated_outs('posenet.npy', True) # GoogLeNet (Trained on Places)
   
    

    adam = Adam(lr=0.001, clipvalue=1.5)
   
    #Same compilation for both nets, dense layers have sema name.
    model.compile(optimizer=adam, loss={'roll': net.euc_lossRoll,
                                        'pitch': net.euc_lossPitch,
                                        'yaw': net.euc_lossYaw, 
                                        'altitude': net.euc_lossAlt})

    

    dataset_train, dataset_test = helper_net.getTrainSource()
   
    X_train = np.squeeze(np.array(dataset_train.images))
    y_train = np.squeeze(np.array(dataset_train.speed))

    y_train_roll = y_train[:,0:1]
    y_train_pitch = y_train[:,1:2]
    y_train_yaw = y_train[:,2:3]
    y_train_altitude = y_train[:,3:4]

    X_test = np.squeeze(np.array(dataset_test.images))
    y_test = np.squeeze(np.array(dataset_test.speed))

    y_test_roll = y_test[:,0:1]
    y_test_pitch = y_test[:,1:2]
    y_test_yaw = y_test[:,2:3]
    y_test_altitude = y_test[:,3:4]

    # Setup checkpointing
    #file_name.h5 -> the file where to save data
    #save_best_only -> not to save all the weights just the ones who improves
    #save_weights_only -> if true save the weights, if false save the entire model +  weights
    checkpointer = ModelCheckpoint(filepath="./models/test.h5", verbose=4, save_best_only=True, save_weights_only=True)

    #In this part you specify the labels for training learning and for validation
    history = model.fit(X_train, [y_train_roll, y_train_pitch, y_train_yaw, y_train_altitude],
          batch_size=batch_size,
          epochs=500,
          validation_data=(X_test, [y_test_roll, y_test_pitch, y_test_yaw, y_test_altitude]),
          callbacks=[checkpointer])

    
    #if you save only weghts true in checkpointer, this in redundant, but if is false, and 
    #want to have separete the weights, uncomment this line.
    model.save_weights("./models/test_weights.h5")
    

    #For plot the training and validation losses 
    loss = history.history['loss']
    val_loss = history.history['val_loss']
    epochs = range(1, len(loss) + 1)
    plt.plot(epochs, loss, color='red', label='Training loss')
    plt.plot(epochs, val_loss, color='green', label='Validation loss')
    plt.title('Training and validation loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()
    plt.show()
