import tensorflow as tf
import numpy as np
from tensorflow.keras.layers import Dense, Input, LSTM, Conv1D, MaxPool1D, Concatenate
from tensorflow.keras.initializers import RandomUniform
from train.embedding import Embedding  # 분리된 Embedding 클래스 임포트

class DQN(tf.keras.Model):
    def __init__(self, state_dim, action_size, embedding_dim=64):
        super(DQN, self).__init__()
        self.state_dim = state_dim
        self.action_size = action_size
        self.embedding_dim = embedding_dim
        # 오류 수정: Input 레이어 제거 또는 수정
        # self.input_layer = Input(shape=(state_dim,))  # 수정된 부분
        self.fc1 = Dense(512, activation='relu')
        self.fc2 = Dense(256, activation='relu')
        self.fc3 = Dense(64, activation='relu')
        self.fc_out = Dense(action_size, 
                          kernel_initializer=RandomUniform(-1e-3, 1e-3))
        
    def call(self, x):
        if len(tf.shape(x)) == 1:
            x = tf.expand_dims(x, 0)  # Add batch dimension if missing
        x = self.fc1(x)
        x = self.fc2(x)
        x = self.fc3(x)
        q = self.fc_out(x)
        return q

class DQNIQN(tf.keras.Model):
    def __init__(self, state_dim, action_size, embedding_dim=64):
        super(DQNIQN, self).__init__()
        self.state_dim = state_dim
        self.action_size = action_size
        self.embedding_dim = embedding_dim
        
        # 임베딩 계층
        self.embedding_out = Embedding(self.embedding_dim)
        
        # 특징 추출 계층 - 배치 정규화 추가
        # 첫 번째 레이어
        self.layer1 = tf.keras.layers.Dense(512)
        self.batch_norm1 = tf.keras.layers.BatchNormalization()
        self.activation1 = tf.keras.layers.ReLU()
        
        # 두 번째 레이어
        self.layer2 = tf.keras.layers.Dense(256)
        self.batch_norm2 = tf.keras.layers.BatchNormalization()
        self.activation2 = tf.keras.layers.ReLU()
        
        # 세 번째 레이어
        self.layer3 = tf.keras.layers.Dense(64)
        self.batch_norm3 = tf.keras.layers.BatchNormalization()
        self.activation3 = tf.keras.layers.ReLU()
        
        # 최종 출력 계층
        self.h_fc1 = tf.keras.layers.Dense(64)
        self.batch_norm_fc1 = tf.keras.layers.BatchNormalization()
        self.activation_fc1 = tf.keras.layers.ReLU()
        
        self.value = tf.keras.layers.Dense(self.action_size)
        
    def call(self, state, num_quantile, tau_min, tau_max, training=True):
        # 상태 특징 추출
        state_shape = tf.shape(state)
        batch_size = state_shape[0]

        # 배치 정규화가 적용된 순방향 전파
        x = self.layer1(state)
        x = self.batch_norm1(x, training=training)  # 수정된 부분
        x = self.activation1(x)
        
        x = self.layer2(x)
        x = self.batch_norm2(x, training=training)  # 수정된 부분
        x = self.activation2(x)
        
        x = self.layer3(x)
        x = self.batch_norm3(x, training=training)  # 수정된 부분
        h_flat = self.activation3(x)
        
        h_flat_tile = tf.tile(h_flat, [num_quantile, 1])
        
        # 임베딩 생성
        embedding_out, sample = self.embedding_out(batch_size, num_quantile, tau_min, tau_max)
        
        # 특징과 임베딩 결합
        h_flat_embedding = tf.multiply(h_flat_tile, embedding_out)
        
        # 최종 출력 계산 (배치 정규화 적용)
        x = self.h_fc1(h_flat_embedding)
        x = self.batch_norm_fc1(x, training=training)  # 수정된 부분
        x = self.activation_fc1(x)
        
        logits = self.value(x)
        logits_reshape = tf.reshape(logits, [num_quantile, batch_size, self.action_size])
        
        # 각 행동에 대한 평균 Q값 계산
        Q_action = tf.reduce_mean(logits_reshape, axis=0)

        return logits_reshape, Q_action, sample

class LSTMDQN(tf.keras.Model):
    def __init__(self, state_size, action_size, batch_size=32, sequence_length=100):
        super(LSTMDQN, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        self.batch_size = batch_size
        self.sequence_length = sequence_length
        
        # LSTM 층 정의
        self.lstm1 = LSTM(64, batch_input_shape=(batch_size, sequence_length, 3), return_sequences=False)
        self.lstm2 = LSTM(64, batch_input_shape=(batch_size, sequence_length, 4), return_sequences=False)
        self.lstm3 = LSTM(64, batch_input_shape=(batch_size, sequence_length, 4), return_sequences=False)
        
        # CNN 층 정의
        self.conv1 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.conv2 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.maxpool = MaxPool1D(pool_size=8, strides=1)
        
        self.conv3 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        
        self.conv4 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.conv5 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.maxpool2 = MaxPool1D(pool_size=5, strides=1)
        
        # FC 층 정의
        self.fc1 = Dense(48, activation='relu')
        self.fc_out = Dense(action_size, kernel_initializer=RandomUniform(-1e-3, 1e-3))
        
    def call(self, x):
        # 차량 상태 처리
        l1_outputs = []
        for i in range(8):  # 8대의 차량
            start_idx = i * 3
            l1_outputs.append(self.lstm1(x[:, :, start_idx:start_idx+3]))
        
        # 자차량 상태 처리
        l2 = self.lstm2(x[:, :, 24:28])
        
        # 공간 상태 처리
        l3_outputs = []
        for i in range(5):  # 5개 공간
            start_idx = 28 + i * 4
            l3_outputs.append(self.lstm3(x[:, :, start_idx:start_idx+4]))
        
        # 차량 상태 결합 및 처리
        h_v = Concatenate(axis=1)([tf.expand_dims(output, axis=1) for output in l1_outputs])
        h_v = self.conv1(h_v)
        h_v = self.conv2(h_v)
        h_1 = self.maxpool(h_v)
        
        # 자차량 상태 처리
        h_EV = tf.expand_dims(l2, axis=1)
        h_2 = self.conv3(h_EV)
        
        # 공간 상태 결합 및 처리
        h_s = Concatenate(axis=1)([tf.expand_dims(output, axis=1) for output in l3_outputs])
        h_s = self.conv4(h_s)
        h_s = self.conv5(h_s)
        h_3 = self.maxpool2(h_s)
        
        # 모든 처리된 상태 결합
        x = Concatenate(axis=2)([h_1, h_2, h_3])
        x = tf.squeeze(x, axis=1)
        
        # 최종 처리
        x = self.fc1(x)
        q = self.fc_out(x)
        
        return q