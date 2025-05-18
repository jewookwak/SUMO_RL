import tensorflow as tf
from tensorflow.keras.layers import Dense, Input, LSTM, Conv1D, MaxPool1D, Concatenate
from tensorflow.keras.initializers import RandomUniform

class LSTMDQN(tf.keras.Model):
    def __init__(self, state_size, action_size, batch_size=32, sequence_length=100):
        super(LSTMDQN, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        self.batch_size = batch_size
        self.sequence_length = sequence_length
        
        # LSTM layers for vehicle states
        self.lstm1 = LSTM(64, batch_input_shape=(batch_size, sequence_length, 3), return_sequences=False)
        # LSTM layer for ego vehicle state
        self.lstm2 = LSTM(64, batch_input_shape=(batch_size, sequence_length, 4), return_sequences=False)
        # LSTM layer for space states
        self.lstm3 = LSTM(64, batch_input_shape=(batch_size, sequence_length, 4), return_sequences=False)
        
        # Convolutional layers for processing hidden states
        self.conv1 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.conv2 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.maxpool = MaxPool1D(pool_size=8, strides=1)
        
        self.conv3 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        
        self.conv4 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.conv5 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.maxpool2 = MaxPool1D(pool_size=5, strides=1)
        
        # Fully connected layers
        self.fc1 = Dense(48, activation='relu')
        self.fc_out = Dense(action_size, kernel_initializer=RandomUniform(-1e-3, 1e-3))
        
    def call(self, x):
        # Process vehicle states
        l1_1 = self.lstm1(x[:, :, 0:3])
        l1_2 = self.lstm1(x[:, :, 3:6])
        l1_3 = self.lstm1(x[:, :, 6:9])
        l1_4 = self.lstm1(x[:, :, 9:12])
        l1_5 = self.lstm1(x[:, :, 12:15])
        l1_6 = self.lstm1(x[:, :, 15:18])
        l1_7 = self.lstm1(x[:, :, 18:21])
        l1_8 = self.lstm1(x[:, :, 21:24])
        
        # Process ego vehicle state
        l2 = self.lstm2(x[:, :, 24:28])
        
        # Process space states
        l3_1 = self.lstm3(x[:, :, 28:32])
        l3_2 = self.lstm3(x[:, :, 32:36])
        l3_3 = self.lstm3(x[:, :, 36:40])
        l3_4 = self.lstm3(x[:, :, 40:44])
        l3_mid = self.lstm3(x[:, :, 44:48])
        
        # Concatenate and process vehicle hidden states
        h_v = Concatenate(axis=1)([
            tf.expand_dims(l1_1, axis=1), 
            tf.expand_dims(l1_2, axis=1), 
            tf.expand_dims(l1_3, axis=1), 
            tf.expand_dims(l1_4, axis=1), 
            tf.expand_dims(l1_5, axis=1), 
            tf.expand_dims(l1_6, axis=1), 
            tf.expand_dims(l1_7, axis=1), 
            tf.expand_dims(l1_8, axis=1)
        ])
        h_v = self.conv1(h_v)
        h_v = self.conv2(h_v)
        h_1 = self.maxpool(h_v)
        
        # Process ego vehicle hidden state
        h_EV = tf.expand_dims(l2, axis=1)
        h_2 = self.conv3(h_EV)
        
        # Concatenate and process space hidden states
        h_s = Concatenate(axis=1)([
            tf.expand_dims(l3_1, axis=1), 
            tf.expand_dims(l3_2, axis=1), 
            tf.expand_dims(l3_3, axis=1), 
            tf.expand_dims(l3_4, axis=1),
            tf.expand_dims(l3_mid, axis=1)
        ])
        h_s = self.conv4(h_s)
        h_s = self.conv5(h_s)
        h_3 = self.maxpool2(h_s)
        
        # Concatenate all processed hidden states
        x = Concatenate(axis=2)([h_1, h_2, h_3])
        x = tf.squeeze(x, axis=1)
        
        # Final processing
        x = self.fc1(x)
        q = self.fc_out(x)
        
        return q