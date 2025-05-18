# train/lstm_prev_nstep_autoencoder_iqn_agent.py
import os
import numpy as np
import tensorflow as tf
from tensorflow.keras.layers import Dense, Input, LSTM, Conv1D, MaxPool1D, Concatenate, Lambda
from tensorflow.keras.optimizers.legacy import Adam
from train.replay_buffer import ReplayBuffer, PrioritizedReplayBuffer, NStepMemory
from train.network.LSTM_autoencoder_network import EnhancedLSTMDQN
import random
from collections import deque
import time


class LSTMPrevNStepAutoencoderIQNAgent:
    """LSTM-DQN agent with Autoencoder and Implicit Quantile Networks, optimized for previous state sequence."""
    
    def __init__(self, state_size, action_size, config):
        """Initialize agent."""
        self.state_size = state_size
        self.action_size = action_size
        self.config = config
        
        # Initialize memory buffer
        self.memory = PrioritizedReplayBuffer(config.MEMORY_SIZE, 
                                              alpha=config.PER_ALPHA, 
                                              beta=config.PER_BETA, 
                                              beta_increment=config.PER_BETA_INCREMENT,
                                              epsilon=config.PER_EPSILON)
        self.n_step_memory = NStepMemory(maxlen=config.N_STEP)
        
        # Implicit Quantile Networks parameters
        self.embedding_dim = config.IQN_EMBEDDING_DIM
        self.action_quantiles = config.IQN_ACTION_QUANTILES
        self.action_tau_min = config.IQN_ACTION_TAU_MIN
        self.action_tau_max = config.IQN_ACTION_TAU_MAX
        self.train_quantiles = config.IQN_TRAIN_QUANTILES
        self.train_tau_min = config.IQN_TRAIN_TAU_MIN
        self.train_tau_max = config.IQN_TRAIN_TAU_MAX
        self.huber_threshold = config.IQN_HUBER_THRESHOLD
        
        # Sequence settings - 이전 상태 시퀀스 길이 사용
        self.sequence_length = config.PREV_SEQUENCE_LENGTH
        self.batch_size = config.BATCH_SIZE
        
        # Training parameters
        self.epsilon = config.EPSILON
        self.epsilon_decay = config.EPSILON_DECAY
        self.epsilon_min = config.EPSILON_MIN
        self.discount_factor = config.DISCOUNT_FACTOR
        self.learning_rate = config.LEARNING_RATE
        self.n_step = config.N_STEP

        # 에피소드 카운터와 입실론 감소 주기 설정
        self.episode_counter = 0
        self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소
        
        # 오토인코더 관련 설정
        self.is_autoencoder_pretrained = False  # 오토인코더 사전 훈련 여부
        
        # Create main and target networks with autoencoder
        self.model = EnhancedLSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
        self.target_model = EnhancedLSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
        
        # Optimizers
        self.optimizer = Adam(learning_rate=self.learning_rate, epsilon=1e-2/self.batch_size)
        self.recon_optimizer = Adam(learning_rate=self.learning_rate)
        
        # Initialize target model with same weights
        self.update_target_model(initial_update=True)
        
    def update_target_model(self, initial_update=False):
        """Update target model with weights from main model."""
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
        """
        Choose action using epsilon-greedy policy
        
        Args:
            state_memory: Current state sequence history
            
        Returns:
            int: Selected action
        """
        if np.all(state_memory[0] == 5):  # 시퀀스가 비어있는지 확인
            return 5
        
        state = np.expand_dims(state_memory, axis=0)
        if np.random.rand() <= self.epsilon:
            return np.random.randint(self.action_size)
        else:
            # For IQN combined with autoencoder, we use the forward pass of the model
            state_tensor = tf.convert_to_tensor(state, dtype=tf.float32)
            q_values = self.model(state_tensor)
            
            # Return action with highest mean Q-value
            return int(np.argmax(q_values[0]))
    
    def append_sample(self, state, action, reward, next_state, done):
        """
        Add experience to replay buffer
        
        Args:
            state: Current state with sequence history
            action: Action taken
            reward: Reward received
            next_state: Next state with sequence history
            done: Whether episode is done
        """
        # 시퀀스가 비어있는지 확인
        if np.all(state[0] == 5):
            return
            
        # Add to n-step buffer
        self.n_step_memory.append(state, next_state, reward, done, action)
        
        # Process n-step returns if we have enough transitions
        n_step_sample = self.n_step_memory.sample()
        if n_step_sample:
            # Calculate n-step return
            n_step_reward = 0
            for i in range(self.n_step):
                if i < len(n_step_sample['reward']):
                    n_step_reward += self.discount_factor**i * n_step_sample['reward'][i]
            
            # Get the initial state and final next_state for n-step
            initial_state = n_step_sample['state'][0]
            final_next_state = n_step_sample['next_state'][-1]
            final_done = n_step_sample['done'][-1]
            action = n_step_sample['action'][0]
            
            # For prioritized replay, we need an initial estimate of the TD error
            state_tensor = tf.convert_to_tensor(np.expand_dims(initial_state, axis=0), dtype=tf.float32)
            next_state_tensor = tf.convert_to_tensor(np.expand_dims(final_next_state, axis=0), dtype=tf.float32)
            
            current_q = self.model(state_tensor)[0][action]
            next_q = self.target_model(next_state_tensor)[0][np.argmax(self.model(next_state_tensor)[0])]
            
            if final_done:
                target = n_step_reward
            else:
                target = n_step_reward + self.discount_factor**self.n_step * next_q
                
            td_error = abs(current_q - target)
            
            # Add experience to prioritized replay buffer
            self.memory.add(td_error, (initial_state, action, n_step_reward, final_next_state, final_done))
    
    def train_model(self):
        """
        Train model using a batch from experience replay
        
        Returns:
            float: Loss value
        """
        if self.memory.tree.n_entries < self.batch_size:
            return 0.0
        
        # Sample batch and importance weights from PER
        batch, indices, is_weights = self.memory.sample(self.batch_size)
        
        states = np.array([sample[0] for sample in batch])
        actions = np.array([sample[1] for sample in batch])
        rewards = np.array([sample[2] for sample in batch])
        next_states = np.array([sample[3] for sample in batch])
        dones = np.array([sample[4] for sample in batch])
        
        # DQN 부분 변수만 가져오기 (오토인코더 제외)
        dqn_vars = []
        for var in self.model.trainable_variables:
            if not any(name in var.name for name in ['vehicle_autoencoder', 'ego_autoencoder', 'space_autoencoder']):
                dqn_vars.append(var)
        
        # Convert to tensors
        states_tensor = tf.convert_to_tensor(states, dtype=tf.float32)
        next_states_tensor = tf.convert_to_tensor(next_states, dtype=tf.float32)
        rewards_tensor = tf.convert_to_tensor(rewards, dtype=tf.float32)
        actions_tensor = tf.convert_to_tensor(actions, dtype=tf.int32)
        dones_tensor = tf.convert_to_tensor(dones, dtype=tf.float32)
        is_weights_tensor = tf.convert_to_tensor(is_weights, dtype=tf.float32)
        
        # IQN specific loss calculation using quantile regression
        with tf.GradientTape() as tape:
            tape.watch(dqn_vars)
            
            # Get current Q-values and next Q-values
            current_q = self.model(states_tensor)
            next_q = self.model(next_states_tensor)
            target_q = self.target_model(next_states_tensor)
            
            # Get best actions based on current network
            best_actions = tf.cast(tf.argmax(next_q, axis=1), tf.int32)  # Cast to int32
            
            # Gather Q-values for actions taken
            action_masks = tf.one_hot(actions_tensor, self.action_size)
            gathered_q = tf.reduce_sum(current_q * action_masks, axis=1)
            
            # Compute target for each batch element
            batch_idx = tf.range(self.batch_size, dtype=tf.int32)  # Specify dtype
            target_values = rewards_tensor + (1 - dones_tensor) * self.discount_factor * tf.gather_nd(
                target_q, 
                tf.stack([batch_idx, best_actions], axis=1)
            )
            
            # Compute quantile regression loss (Huber loss)
            td_errors = target_values[:, tf.newaxis] - gathered_q
            
            # Ensure consistent data types for huber loss calculation
            td_errors = tf.cast(td_errors, tf.float32)
            huber_threshold = tf.constant(self.huber_threshold, dtype=tf.float32)
            
            huber_loss = tf.where(
                tf.abs(td_errors) <= huber_threshold,
                0.5 * tf.square(td_errors),
                huber_threshold * (tf.abs(td_errors) - 0.5 * huber_threshold)
            )
            
            # Apply importance sampling weights from PER
            weighted_loss = huber_loss * is_weights_tensor[:, tf.newaxis]
            
            # Total loss
            loss = tf.reduce_mean(weighted_loss)
        
        # Compute gradients and update network (DQN 부분만)
        gradients = tape.gradient(loss, dqn_vars)
        self.optimizer.apply_gradients(zip(gradients, dqn_vars))
        
        # Update priorities in replay buffer
        td_errors_np = td_errors.numpy()
        for i in range(self.batch_size):
            # Make sure we're using a scalar value for the error
            error_val = float(np.abs(td_errors_np[i][0]))
            self.memory.update(indices[i], error_val)
        
        return loss.numpy()
    
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
    
    def end_episode(self, episode_reward):
        """
        Perform end-of-episode operations
        
        Args:
            episode_reward: Total reward for the episode
            
        Returns:
            float: Updated epsilon value
        """
        self.episode_counter += 1
        
        # 10 에피소드마다 입실론 감소
        if self.episode_counter % self.epsilon_update_frequency == 0 and self.epsilon > self.epsilon_min:
            old_epsilon = self.epsilon
            self.epsilon *= self.epsilon_decay
            print(f"Epsilon decreased from {old_epsilon:.4f} to {self.epsilon:.4f}")
        
        # n-step 메모리 초기화
        self.n_step_memory.clear()
        
        return self.epsilon
    
    def save_model(self, path):
        """Save model weights to file."""
        if not os.path.exists(path):
            os.makedirs(path)
        self.model.save_weights(os.path.join(path, 'lstm_prev_nstep_autoencoder_iqn_model.h5'))
        
    def load_model(self, path):
        """Load model weights from file."""
        self.model.load_weights(os.path.join(path, 'lstm_prev_nstep_autoencoder_iqn_model.h5'))
        self.update_target_model(initial_update=True)