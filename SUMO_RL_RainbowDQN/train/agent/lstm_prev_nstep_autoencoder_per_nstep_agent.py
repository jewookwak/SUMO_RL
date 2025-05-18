# train/lstm_prev_nstep_autoencoder_per_nstep_agent.py
import tensorflow as tf
import numpy as np
from collections import deque
import random
from tensorflow.keras.optimizers.legacy import Adam
from train.network.LSTM_autoencoder_network import EnhancedLSTMDQN
from train.replay_buffer import PrioritizedReplayBuffer, NStepMemory

class LSTMPrevNStepAutoencoderPERNStepAgent:
    def __init__(self, state_size, action_size, config):
        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size
        self.config = config

        # 하이퍼파라미터 설정
        self.discount_factor = config.DISCOUNT_FACTOR
        self.learning_rate = config.LEARNING_RATE
        self.epsilon = config.EPSILON
        self.epsilon_decay = config.EPSILON_DECAY
        self.epsilon_min = config.EPSILON_MIN
        self.batch_size = config.BATCH_SIZE
        self.n_step = config.N_STEP
        self.sequence_length = config.PREV_SEQUENCE_LENGTH  # 이전 상태 시퀀스 길이 사용
        
        # 에피소드 카운터와 입실론 감소 주기 설정
        self.episode_counter = 0
        self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소

        # 모델 생성 (오토인코더가 포함된 LSTM DQN)
        self.model = EnhancedLSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
        self.target_model = EnhancedLSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
        
        # 옵티마이저 (legacy 버전 사용)
        self.optimizer = Adam(learning_rate=self.config.LEARNING_RATE)
        
        # 재구성 손실을 위한 별도의 옵티마이저
        self.recon_optimizer = Adam(learning_rate=self.config.LEARNING_RATE)
        
        # n-step 메모리와 PER 버퍼 초기화
        self.n_step_memory = NStepMemory(maxlen=self.n_step)
        self.memory = PrioritizedReplayBuffer(capacity=config.MEMORY_SIZE)

        # 오토인코더 관련 설정
        self.is_autoencoder_pretrained = False  # 오토인코더 사전 훈련 여부
        
        # 타깃 모델 초기화
        self.update_target_model(initial_update=True)

    def update_target_model(self, initial_update=False):
        """타깃 모델을 모델의 가중치로 업데이트"""
        try:
            # 레이어별 가중치 복사 접근법
            print("레이어별 가중치 복사 시도 중...")
            
            # 오토인코더 레이어 복사
            if initial_update:
                for src_encoder_name, tgt_encoder_name in [
                    ('vehicle_autoencoder', 'vehicle_autoencoder'),
                    ('ego_autoencoder', 'ego_autoencoder'),
                    ('space_autoencoder', 'space_autoencoder')
                ]:
                    src_encoder = getattr(self.model, src_encoder_name)
                    tgt_encoder = getattr(self.target_model, src_encoder_name)
                    
                    for i, (src_layer, tgt_layer) in enumerate(zip(src_encoder.layers, tgt_encoder.layers)):
                        tgt_layer.set_weights(src_layer.get_weights())
                        print(f"  {src_encoder_name} 레이어 {i} 복사 완료")
            
            if not initial_update:
                # CNN 및 기타 레이어 복사
                for layer_name in ['conv1', 'conv2', 'maxpool', 'conv3', 'conv4', 'conv5', 'maxpool2', 'fc1', 'fc_out']:
                    src_layer = getattr(self.model, layer_name)
                    tgt_layer = getattr(self.target_model, layer_name)
                    tgt_layer.set_weights(src_layer.get_weights())
                    print(f"  {layer_name} 레이어 복사 완료")
                
        except Exception as e:
            print(f'initial update is {initial_update}')
            print(f"레이어별 가중치 복사 실패: {e}")
            print("전체 모델 가중치 복사 시도는 생략합니다.")

    def get_action(self, state_memory):
        """입실론 탐욕 정책으로 행동 선택"""
        if np.all(state_memory[0] == 5):  # 시퀀스가 비어있는지 확인
            return 5
        
        state = np.expand_dims(state_memory, axis=0)
        
        if np.random.rand() <= self.epsilon:
            return np.random.randint(self.action_size)
        else:
            q_value = self.model(state)
            return np.argmax(q_value[0])

    def append_sample(self, state, action, reward, next_state, done):
        """n-step 메모리에 경험 추가 및 TD 오류 계산"""
        # 시퀀스가 비어있는지 확인
        if np.all(state[0] == 5):
            return
            
        self.n_step_memory.append(state, next_state, reward, done, action)
        n_step_sample = self.n_step_memory.sample()
        
        if n_step_sample is not None:
            state = n_step_sample['state'][0]
            next_state = n_step_sample['next_state'][-1]
            reward = n_step_sample['reward']
            done = n_step_sample['done'][-1]
            action = n_step_sample['action'][0]

            state_tensor = tf.convert_to_tensor([state], dtype=tf.float32)
            next_state_tensor = tf.convert_to_tensor([next_state], dtype=tf.float32)

            # 현재 상태에 대한 Q값 예측
            main_current_q = np.array(self.model(state_tensor))[0]
            
            # 다음 상태에 대한 Q값 예측 (더블 DQN)
            main_next_q = np.array(self.model(next_state_tensor))[0]
            target_next_q = np.array(self.target_model(next_state_tensor))[0]
            
            # 더블 DQN: 메인 네트워크로 행동 선택
            next_action = np.argmax(main_next_q)
            target_next_q_value = target_next_q[next_action]
            
            # 현재 Q값
            current_q = main_current_q[action]

            # n-step 리턴 계산
            n_step_return = np.sum([np.power(self.discount_factor, i) * r for i, r in enumerate(reward)])

            # TD 오류 계산
            td_error = np.abs(n_step_return + np.power(self.discount_factor, self.n_step) * target_next_q_value * (1-done) - current_q)

            # PER 메모리에 추가
            self.memory.add(td_error, (state, action, n_step_return, next_state, done))

    def train_model(self):
        """PER 메모리에서 샘플링하여 모델 업데이트 (DQN 부분만 학습)"""
        if self.memory.tree.n_entries < self.batch_size:
            return 0.0
            
        # PER 메모리에서 배치 샘플링
        mini_batch, idxs, IS_weight = self.memory.sample(self.batch_size)
        mini_batch = np.array(mini_batch)
        
        states = np.array([i[0] for i in mini_batch])
        actions = np.array([i[1] for i in mini_batch])
        rewards = np.array([i[2] for i in mini_batch])
        next_states = np.array([i[3] for i in mini_batch])
        dones = np.array([i[4] for i in mini_batch])

        # DQN 부분 변수만 가져오기 (오토인코더 제외)
        dqn_vars = []
        for var in self.model.trainable_variables:
            if not any(name in var.name for name in ['vehicle_autoencoder', 'ego_autoencoder', 'space_autoencoder']):
                dqn_vars.append(var)

        # 학습 단계
        with tf.GradientTape() as tape:
            tape.watch(dqn_vars)
            
            # 현재 상태에 대한 Q값
            rewards = tf.convert_to_tensor(rewards, dtype=tf.float32)
            actions = tf.convert_to_tensor(actions, dtype=tf.int32)
            dones = tf.convert_to_tensor(dones, dtype=tf.float32)
            
            # 다음 상태에 대한 Q값
            target_q = self.target_model(tf.convert_to_tensor(next_states, dtype=tf.float32))
            main_q = self.model(tf.convert_to_tensor(next_states, dtype=tf.float32))
            main_q = tf.stop_gradient(main_q)
            next_action = tf.argmax(main_q, axis=1)
            target_value = tf.reduce_sum(tf.one_hot(next_action, self.action_size) * target_q, axis=1)
            
            # 타깃 계산 (n-step 보상 + 감마^n * 다음 상태 가치)
            target_value = (1-dones) * np.power(self.discount_factor, self.n_step) * target_value + rewards
            
            # 현재 모델의 Q값 예측
            main_q = self.model(tf.convert_to_tensor(states, dtype=tf.float32))
            main_value = tf.reduce_sum(tf.one_hot(actions, self.action_size) * main_q, axis=1)
            
            # TD 오류 및 손실 계산 (IS 가중치 적용)
            error = tf.square(main_value - target_value) * 0.5
            error = error * tf.convert_to_tensor(IS_weight, dtype=tf.float32)
            error = tf.reduce_mean(error)
            
        # 그래디언트 계산 및 적용 (DQN 부분만)
        dqn_grads = tape.gradient(error, dqn_vars)
        self.optimizer.apply_gradients(zip(dqn_grads, dqn_vars))
        
        # PER 메모리 우선순위 업데이트
        state_value = np.array(self.model(tf.convert_to_tensor(states, dtype=tf.float32)))
        state_value = np.array([sv[a] for a, sv in zip(actions, state_value)])
        
        td_error = np.abs(target_value.numpy() - state_value)
        
        for i in range(self.batch_size):
            idx = idxs[i]
            self.memory.update(idx, td_error[i])
            
        return error.numpy()

    def pretrain_autoencoder(self, train_data, epochs=30):
        """오토인코더 부분을 사전 훈련하는 함수"""
        print("===== 오토인코더 사전 훈련 시작 =====")
        
        # 훈련 데이터에서 각 상태 그룹 추출
        vehicle_data = []
        for i in range(0, 24, 3):
            vehicle_data.append(train_data[:, :, i:i+3])
        
        ego_data = train_data[:, :, 24:28]
        
        space_data = []
        for i in range(28, 48, 4):
            space_data.append(train_data[:, :, i:i+4])
        
        # 훈련 지표 저장용
        history = {
            'vehicle_loss': [],
            'ego_loss': [],
            'space_loss': []
        }
        
        # 에포크별 훈련
        for epoch in range(epochs):
            import time
            start_time = time.time()
            total_vehicle_loss = 0
            total_ego_loss = 0
            total_space_loss = 0
            
            # 1. 차량 상태 오토인코더 훈련
            for v_idx, v_data in enumerate(vehicle_data):
                num_batches = len(v_data) // self.batch_size
                
                for batch in range(num_batches):
                    batch_data = v_data[batch*self.batch_size:(batch+1)*self.batch_size]
                    
                    with tf.GradientTape() as tape:
                        reconstructed, _ = self.model.vehicle_autoencoder(batch_data)
                        loss = tf.reduce_mean(tf.square(batch_data - reconstructed))
                    
                    gradients = tape.gradient(loss, self.model.vehicle_autoencoder.trainable_variables)
                    self.recon_optimizer.apply_gradients(zip(gradients, self.model.vehicle_autoencoder.trainable_variables))
                    
                    total_vehicle_loss += loss.numpy()
            
            avg_vehicle_loss = total_vehicle_loss / (len(vehicle_data) * num_batches) if num_batches > 0 else 0
            history['vehicle_loss'].append(avg_vehicle_loss)
            
            # 2. 자차 상태 오토인코더 훈련
            num_batches = len(ego_data) // self.batch_size
            for batch in range(num_batches):
                batch_data = ego_data[batch*self.batch_size:(batch+1)*self.batch_size]
                
                with tf.GradientTape() as tape:
                    reconstructed, _ = self.model.ego_autoencoder(batch_data)
                    loss = tf.reduce_mean(tf.square(batch_data - reconstructed))
                
                gradients = tape.gradient(loss, self.model.ego_autoencoder.trainable_variables)
                self.recon_optimizer.apply_gradients(zip(gradients, self.model.ego_autoencoder.trainable_variables))
                
                total_ego_loss += loss.numpy()
            
            avg_ego_loss = total_ego_loss / num_batches if num_batches > 0 else 0
            history['ego_loss'].append(avg_ego_loss)
            
            # 3. 공간 상태 오토인코더 훈련
            for s_idx, s_data in enumerate(space_data):
                num_batches = len(s_data) // self.batch_size
                
                for batch in range(num_batches):
                    batch_data = s_data[batch*self.batch_size:(batch+1)*self.batch_size]
                    
                    with tf.GradientTape() as tape:
                        reconstructed, _ = self.model.space_autoencoder(batch_data)
                        loss = tf.reduce_mean(tf.square(batch_data - reconstructed))
                    
                    gradients = tape.gradient(loss, self.model.space_autoencoder.trainable_variables)
                    self.recon_optimizer.apply_gradients(zip(gradients, self.model.space_autoencoder.trainable_variables))
                    
                    total_space_loss += loss.numpy()
            
            avg_space_loss = total_space_loss / (len(space_data) * num_batches) if num_batches > 0 else 0
            history['space_loss'].append(avg_space_loss)
            
            # 진행 상황 출력
            elapsed_time = time.time() - start_time
            print(f"Epoch {epoch+1}/{epochs}, Time: {elapsed_time:.2f}s")
            print(f"  Vehicle Loss: {avg_vehicle_loss:.6f}")
            print(f"  Ego Loss: {avg_ego_loss:.6f}")
            print(f"  Space Loss: {avg_space_loss:.6f}")
        
        print("===== 오토인코더 사전 훈련 완료 =====")
        
        # 오토인코더 가중치 고정
        self.freeze_autoencoder_weights()
        
        # 사전 훈련 완료 플래그 설정
        self.is_autoencoder_pretrained = True
        
        return history

    def freeze_autoencoder_weights(self):
        """오토인코더 가중치를 고정하는 함수"""
        print("오토인코더 가중치 고정 중...")
        
        # 차량 상태 오토인코더 고정
        for layer in self.model.vehicle_autoencoder.layers:
            layer.trainable = False
        
        # 자차 상태 오토인코더 고정
        for layer in self.model.ego_autoencoder.layers:
            layer.trainable = False
        
        # 공간 상태 오토인코더 고정
        for layer in self.model.space_autoencoder.layers:
            layer.trainable = False
            
        # 타겟 모델에도 동일하게 적용
        for layer in self.target_model.vehicle_autoencoder.layers:
            layer.trainable = False
        
        for layer in self.target_model.ego_autoencoder.layers:
            layer.trainable = False
        
        for layer in self.target_model.space_autoencoder.layers:
            layer.trainable = False
        
        print("오토인코더 가중치 고정 완료")
        
    def end_episode(self, episode_reward=None):
        """에피소드 종료 시 호출하는 메서드"""
        self.episode_counter += 1
        
        # 10 에피소드마다 입실론 감소
        if self.episode_counter % self.epsilon_update_frequency == 0 and self.epsilon > self.epsilon_min:
            old_epsilon = self.epsilon
            self.epsilon *= self.epsilon_decay
            print(f"Epsilon decreased from {old_epsilon:.4f} to {self.epsilon:.4f}")
        
        # n-step 메모리 초기화
        self.n_step_memory.clear()
        
        # 현재 입실론 값 반환 (TensorBoard 로깅용)
        return self.epsilon