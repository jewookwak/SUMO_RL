# train/LSTM_autoencoder_network.py
import tensorflow as tf
import numpy as np
from tensorflow.keras.layers import Dense, Input, LSTM, Conv1D, MaxPool1D, Concatenate, RepeatVector, TimeDistributed
from tensorflow.keras.initializers import RandomUniform

class LSTMAutoencoder(tf.keras.Model):
    def __init__(self, input_dim, latent_dim, sequence_length):
        super(LSTMAutoencoder, self).__init__()
        self.input_dim = input_dim
        self.latent_dim = latent_dim
        self.sequence_length = sequence_length
        
        # 인코더: 입력 시퀀스를 압축
        self.encoder_lstm = LSTM(latent_dim, return_state=True)
        
        # 디코더: 압축된 표현을 원래 시퀀스로 복원
        self.decoder_lstm = LSTM(latent_dim, return_sequences=True)
        self.decoder_dense = TimeDistributed(Dense(input_dim))
        
    def encode(self, x):
        # 입력 시퀀스를 인코딩하여 잠재 표현 반환
        _, h, c = self.encoder_lstm(x)
        return h  # 은닉 상태만 반환
    
    def call(self, x):
        # 전체 오토인코더 실행 (인코딩 + 디코딩)
        _, h, c = self.encoder_lstm(x)
        
        # 잠재 표현을 시퀀스 길이에 맞게 반복
        repeated = RepeatVector(self.sequence_length)(h)
        # 디코더 LSTM으로 시퀀스 복원
        decoded = self.decoder_lstm(repeated)
        # 원래 입력 형태로 변환
        reconstructed = self.decoder_dense(decoded)
        
        return reconstructed, h  # 재구성된 시퀀스와 잠재 표현 반환

class EnhancedLSTMDQN(tf.keras.Model):
    def __init__(self, state_size, action_size, batch_size=32, sequence_length=20):
        super(EnhancedLSTMDQN, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        self.batch_size = batch_size
        self.sequence_length = sequence_length
        
        # 압축 비율 설정 (5:1 압축)
        compression_ratio = 5
        
        # 시퀀스 길이를 고려한 잠재 차원 계산
        vehicle_latent_dim = max(4, (sequence_length * 3) // compression_ratio)  # 최소 4차원
        ego_latent_dim = max(4, (sequence_length * 4) // compression_ratio)      # 최소 4차원
        space_latent_dim = max(4, (sequence_length * 4) // compression_ratio)    # 최소 4차원
        
        print(f"Vehicle latent dim: {vehicle_latent_dim}")
        print(f"Ego latent dim: {ego_latent_dim}")
        print(f"Space latent dim: {space_latent_dim}")
        
        # 차량 상태용 오토인코더 (3차원 입력)
        self.vehicle_autoencoder = LSTMAutoencoder(
            input_dim=3, 
            latent_dim=vehicle_latent_dim, 
            sequence_length=sequence_length
        )
        
        # 자차 상태용 오토인코더 (4차원 입력)
        self.ego_autoencoder = LSTMAutoencoder(
            input_dim=4, 
            latent_dim=ego_latent_dim, 
            sequence_length=sequence_length
        )
        
        # 공간 상태용 오토인코더 (4차원 입력)
        self.space_autoencoder = LSTMAutoencoder(
            input_dim=4, 
            latent_dim=space_latent_dim,
            sequence_length=sequence_length
        )
        
        # 특성 추출 컨볼루션 레이어
        self.conv1 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.conv2 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.maxpool = MaxPool1D(pool_size=8, strides=1)
        
        self.conv3 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        
        self.conv4 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.conv5 = Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.maxpool2 = MaxPool1D(pool_size=5, strides=1)
        
        # 최종 예측 레이어
        self.fc1 = Dense(48, activation='relu')
        self.fc_out = Dense(action_size, kernel_initializer=RandomUniform(-1e-3, 1e-3))
    
    def call(self, x, training=False):
        # 오토인코더를 사용한 상태 압축
        vehicle_encodings = []
        
        # 8개의 차량 상태 처리
        for i in range(8):  # 8대의 차량
            start_idx = i * 3
            encoded = self.vehicle_autoencoder.encode(x[:, :, start_idx:start_idx+3])
            vehicle_encodings.append(encoded)
        
        # 자차 상태 처리
        ego_encoded = self.ego_autoencoder.encode(x[:, :, 24:28])
        
        # 5개의 공간 상태 처리
        space_encodings = []
        for i in range(5):  # 5개 공간
            start_idx = 28 + i * 4
            encoded = self.space_autoencoder.encode(x[:, :, start_idx:start_idx+4])
            space_encodings.append(encoded)
        
        # 차량 상태 특성 처리
        h_v = Concatenate(axis=1)([tf.expand_dims(encoding, axis=1) for encoding in vehicle_encodings])
        h_v = self.conv1(h_v)
        h_v = self.conv2(h_v)
        h_1 = self.maxpool(h_v)
        
        # 자차 상태 특성 처리
        h_EV = tf.expand_dims(ego_encoded, axis=1)
        h_2 = self.conv3(h_EV)
        
        # 공간 상태 특성 처리
        h_s = Concatenate(axis=1)([tf.expand_dims(encoding, axis=1) for encoding in space_encodings])
        h_s = self.conv4(h_s)
        h_s = self.conv5(h_s)
        h_3 = self.maxpool2(h_s)
        
        # 모든 특성 통합
        x = Concatenate(axis=2)([h_1, h_2, h_3])
        x = tf.squeeze(x, axis=1)
        
        # 최종 예측
        x = self.fc1(x)
        q = self.fc_out(x)
        
        return q