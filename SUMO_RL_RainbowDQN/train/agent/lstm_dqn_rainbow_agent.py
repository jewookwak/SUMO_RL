# train/lstm_dqn_rainbow_agent.py
import tensorflow as tf
import numpy as np
from collections import deque
import random
from tensorflow.keras.optimizers import Adam
from train.replay_buffer import PrioritizedReplayBuffer, NStepMemory

class NoisyDense(tf.keras.layers.Layer):
    """Noisy Network 구현 (파라미터에 노이즈를 추가하여 탐색 강화)"""
    def __init__(self, units, activation=None, sigma_init=0.5):
        super(NoisyDense, self).__init__()
        self.units = units
        self.sigma_init = sigma_init
        
        # 문자열 activation을 함수로 변환
        if isinstance(activation, str):
            self.activation = tf.keras.activations.get(activation)
        else:
            self.activation = activation
        
    def build(self, input_shape):
        self.w_mu = self.add_weight(
            shape=(input_shape[-1], self.units),
            initializer=tf.keras.initializers.GlorotUniform(),
            trainable=True,
            name="w_mu")
        self.w_sigma = self.add_weight(
            shape=(input_shape[-1], self.units),
            initializer=tf.keras.initializers.Constant(self.sigma_init / np.sqrt(input_shape[-1])),
            trainable=True,
            name="w_sigma")
        
        self.b_mu = self.add_weight(
            shape=(self.units,),
            initializer=tf.keras.initializers.Zeros(),
            trainable=True,
            name="b_mu")
        self.b_sigma = self.add_weight(
            shape=(self.units,),
            initializer=tf.keras.initializers.Constant(self.sigma_init / np.sqrt(input_shape[-1])),
            trainable=True,
            name="b_sigma")
            
    def call(self, inputs, training=None):
        if training:
            # 입력과 출력 크기에 맞는 랜덤 노이즈 생성
            p = tf.random.normal([inputs.shape[-1], self.units])
            q = tf.random.normal([1, self.units])
            f_p = tf.math.sign(p) * tf.math.sqrt(tf.math.abs(p))
            f_q = tf.math.sign(q) * tf.math.sqrt(tf.math.abs(q))
            
            # 노이즈가 있는 가중치와 편향 계산
            w_epsilon = f_p * f_q
            b_epsilon = tf.squeeze(f_q)
            
            w = self.w_mu + self.w_sigma * w_epsilon
            b = self.b_mu + self.b_sigma * b_epsilon
            
            # 선형 변환 적용
            ret = tf.matmul(inputs, w) + b
        else:
            # 추론 시에는 평균 가중치만 사용
            ret = tf.matmul(inputs, self.w_mu) + self.b_mu
            
        # 활성화 함수 적용
        if self.activation is not None:
            ret = self.activation(ret)
            
        return ret

class Embedding(tf.keras.Model):
    def __init__(self, embedding_dim):
        super(Embedding, self).__init__()
        self.layer = tf.keras.layers.Dense(embedding_dim, activation='relu')
        self.embedding_dim = embedding_dim

    def call(self, batch_size, num_quantile, tau_min, tau_max):
        # 배치 크기가 텐서인 경우 스칼라로 변환
        if isinstance(batch_size, tf.Tensor):
            batch_size = tf.cast(batch_size, tf.int32)
            
        # 임베딩 샘플 생성
        # [batch_size * num_quantile, 1] 형태로 표본 생성
        total_samples = batch_size * num_quantile
        sample = tf.random.uniform(
            [total_samples, 1],
            minval=tau_min, maxval=tau_max, dtype=tf.float32)
        
        # [batch_size * num_quantile, embedding_dim] 형태로 타일링
        sample_tile = tf.tile(sample, [1, self.embedding_dim])
        
        # 코사인 임베딩 계산
        embedding = tf.cos(
            tf.cast(tf.range(0, self.embedding_dim, 1), tf.float32) * np.pi * sample_tile)
        
        # 임베딩 레이어 통과
        embedding_out = self.layer(embedding)
        
        return embedding_out, sample

class LSTMRainbowDQN(tf.keras.Model):
    def __init__(self, state_size, action_size, batch_size=32, sequence_length=100, embedding_dim=64, use_noisy=True):
        super(LSTMRainbowDQN, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        self.batch_size = batch_size
        self.sequence_length = sequence_length
        self.embedding_dim = embedding_dim
        self.use_noisy = use_noisy
        
        # 임베딩 계층
        self.embedding_out = Embedding(self.embedding_dim)
        
        # LSTM 계층 정의
        self.lstm1 = tf.keras.layers.LSTM(64, batch_input_shape=(batch_size, sequence_length, 3), return_sequences=False)
        self.lstm2 = tf.keras.layers.LSTM(64, batch_input_shape=(batch_size, sequence_length, 4), return_sequences=False)
        self.lstm3 = tf.keras.layers.LSTM(64, batch_input_shape=(batch_size, sequence_length, 4), return_sequences=False)
        
        # CNN 계층 정의
        self.conv1 = tf.keras.layers.Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.conv2 = tf.keras.layers.Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.maxpool = tf.keras.layers.MaxPool1D(pool_size=8, strides=1)
        
        self.conv3 = tf.keras.layers.Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        
        self.conv4 = tf.keras.layers.Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.conv5 = tf.keras.layers.Conv1D(filters=32, kernel_size=1, strides=1, activation='relu')
        self.maxpool2 = tf.keras.layers.MaxPool1D(pool_size=5, strides=1)
        
        # 특징 추출 계층
        self.fc1 = tf.keras.layers.Dense(64, activation='relu')
        
        # Dueling Network 구조 (IQN와 결합)
        if use_noisy:
            # Noisy Network 사용
            self.value_hidden = NoisyDense(64, activation='relu')
            self.value = NoisyDense(1)
            self.advantage_hidden = NoisyDense(64, activation='relu')
            self.advantage = NoisyDense(self.action_size)
        else:
            # 일반 레이어 사용
            self.value_hidden = tf.keras.layers.Dense(64, activation='relu')
            self.value = tf.keras.layers.Dense(1)
            self.advantage_hidden = tf.keras.layers.Dense(64, activation='relu')
            self.advantage = tf.keras.layers.Dense(self.action_size)
    
    def call(self, x, num_quantile=8, tau_min=0.0, tau_max=1.0, training=False):
        batch_size = tf.shape(x)[0]
        
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
        h_v = tf.keras.layers.Concatenate(axis=1)([tf.expand_dims(output, axis=1) for output in l1_outputs])
        h_v = self.conv1(h_v)
        h_v = self.conv2(h_v)
        h_1 = self.maxpool(h_v)
        
        # 자차량 상태 처리
        h_EV = tf.expand_dims(l2, axis=1)
        h_2 = self.conv3(h_EV)
        
        # 공간 상태 결합 및 처리
        h_s = tf.keras.layers.Concatenate(axis=1)([tf.expand_dims(output, axis=1) for output in l3_outputs])
        h_s = self.conv4(h_s)
        h_s = self.conv5(h_s)
        h_3 = self.maxpool2(h_s)
        
        # 모든 처리된 상태 결합
        h_combined = tf.keras.layers.Concatenate(axis=2)([h_1, h_2, h_3])
        h_flat = tf.squeeze(h_combined, axis=1)
        
        # 특징 추출
        features = self.fc1(h_flat)
        
        # 임베딩 생성
        embedding_out, sample = self.embedding_out(batch_size, num_quantile, tau_min, tau_max)
        
        # 각 배치 항목을 num_quantile 개로 확장하고 타일링
        features_expanded = tf.expand_dims(features, axis=1)  # [batch_size, 1, feature_dim]
        features_tiled = tf.tile(features_expanded, [1, num_quantile, 1])  # [batch_size, num_quantile, feature_dim]
        features_reshaped = tf.reshape(features_tiled, [batch_size * num_quantile, -1])  # [batch_size * num_quantile, feature_dim]
        
        # 특징과 임베딩 결합 (요소별 곱셈)
        combined_features = features_reshaped * embedding_out
        
        # Dueling Network 구조 적용
        value_hidden = self.value_hidden(combined_features, training=training)
        value = self.value(value_hidden, training=training)
        
        advantage_hidden = self.advantage_hidden(combined_features, training=training)
        advantage = self.advantage(advantage_hidden, training=training)
        
        # V(s) + A(s,a) - mean(A(s,a)) 계산
        value = tf.reshape(value, [-1, 1])
        mean_advantage = tf.reduce_mean(advantage, axis=1, keepdims=True)
        logits = value + (advantage - mean_advantage)
        
        # 분위수에 따라 재구성
        logits_reshape = tf.reshape(logits, [num_quantile, batch_size, self.action_size])
        
        # 각 행동에 대한 평균 Q값 계산
        Q_action = tf.reduce_mean(logits_reshape, axis=0)

        return logits_reshape, Q_action, sample

class LSTMDQNRainbowAgent:
    def __init__(self, state_size, action_size, config):
        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size
        self.config = config
        self.sequence_length = config.SEQUENCE_LENGTH
        self.batch_size = config.BATCH_SIZE
        self.n_step = config.N_STEP

        # 하이퍼파라미터 설정
        self.discount_factor = config.DISCOUNT_FACTOR
        self.learning_rate = config.LEARNING_RATE
        self.epsilon = config.EPSILON
        self.epsilon_decay = config.EPSILON_DECAY
        self.epsilon_min = config.EPSILON_MIN
        
        # IQN 하이퍼파라미터
        self.embedding_dim = config.IQN_EMBEDDING_DIM
        self.get_action_num_quantile = config.IQN_ACTION_QUANTILES
        self.get_action_tau_min = config.IQN_ACTION_TAU_MIN
        self.get_action_tau_max = config.IQN_ACTION_TAU_MAX
        self.train_num_quantile = config.IQN_TRAIN_QUANTILES
        self.train_tau_min = config.IQN_TRAIN_TAU_MIN
        self.train_tau_max = config.IQN_TRAIN_TAU_MAX
        self.huber_threshold = config.IQN_HUBER_THRESHOLD
        
        # Rainbow 추가 하이퍼파라미터
        self.use_noisy_net = config.USE_NOISY_NETWORK if hasattr(config, 'USE_NOISY_NETWORK') else True
        self.noisy_sigma_init = config.NOISY_SIGMA_INIT if hasattr(config, 'NOISY_SIGMA_INIT') else 0.5
        self.target_update_freq = config.TARGET_UPDATE_FREQ if hasattr(config, 'TARGET_UPDATE_FREQ') else 500
        
        # PER 하이퍼파라미터
        self.per_alpha = config.PER_ALPHA if hasattr(config, 'PER_ALPHA') else 0.6
        self.per_beta = config.PER_BETA if hasattr(config, 'PER_BETA') else 0.4
        self.per_beta_increment = config.PER_BETA_INCREMENT if hasattr(config, 'PER_BETA_INCREMENT') else 0.001
        self.per_epsilon = config.PER_EPSILON if hasattr(config, 'PER_EPSILON') else 0.01
        
        # 에피소드 카운터와 입실론 감소 주기 설정
        self.episode_counter = 0
        self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소
        self.train_step_counter = 0

        # 모델과 타깃 모델 생성
        self.model = LSTMRainbowDQN(
            state_size, action_size, self.batch_size, self.sequence_length, 
            self.embedding_dim, self.use_noisy_net
        )
        self.target_model = LSTMRainbowDQN(
            state_size, action_size, self.batch_size, self.sequence_length, 
            self.embedding_dim, self.use_noisy_net
        )
        self.optimizer = Adam(learning_rate=self.learning_rate, epsilon=1e-2/self.batch_size)

        # n-step 메모리와 PER 버퍼 초기화
        self.n_step_memory = NStepMemory(maxlen=self.n_step)
        self.memory = PrioritizedReplayBuffer(
            capacity=config.MEMORY_SIZE,
            alpha=self.per_alpha,
            beta=self.per_beta,
            beta_increment=self.per_beta_increment,
            epsilon=self.per_epsilon
        )

        # 타깃 모델 초기화
        self.update_target_model()
        
        # 모델 초기화를 위한 더미 호출
        # 초기 입력 형태를 맞춰서 더미 텐서 생성
        dummy_state = np.zeros((self.batch_size, self.sequence_length, self.state_size))
        dummy_tensor = tf.convert_to_tensor(dummy_state, dtype=tf.float32)
        
        try:
            # 모델 더미 호출
            self.model(
                dummy_tensor, 
                self.train_num_quantile, 
                self.train_tau_min,
                self.train_tau_max
            )
            self.target_model(
                dummy_tensor, 
                self.train_num_quantile,
                self.train_tau_min, 
                self.train_tau_max
            )
            print("모델 초기화 성공")
        except Exception as e:
            print(f"모델 초기화 중 오류 발생: {e}")
            raise e

    def update_target_model(self):
        """타깃 모델을 모델의 가중치로 업데이트"""
        self.target_model.set_weights(self.model.get_weights())

    def get_action(self, state_memory):
        """행동 선택 - Noisy Network 사용 시 epsilon-greedy 사용 안함"""
        if np.all(state_memory[0] == 0) or np.all(state_memory[0] == 5):  # 시퀀스가 비어있는지 확인
            return 5
        
        state = np.expand_dims(state_memory, axis=0)
        state_tensor = tf.convert_to_tensor(state, dtype=tf.float32)
        
        if self.use_noisy_net:
            # Noisy Network 사용 시 노이즈를 통한 내재적 탐색
            _, q_value, _ = self.model(
                state_tensor,
                self.get_action_num_quantile,
                self.get_action_tau_min,
                self.get_action_tau_max,
                training=True)  # 학습 모드로 호출하여 노이즈 적용
            return int(np.argmax(q_value[0]))
        else:
            # 기존 epsilon-greedy 방식
            if np.random.rand() <= self.epsilon:
                return np.random.randint(self.action_size)
            else:
                _, q_value, _ = self.model(
                    state_tensor,
                    self.get_action_num_quantile,
                    self.get_action_tau_min,
                    self.get_action_tau_max)
                return int(np.argmax(q_value[0]))

    def append_sample(self, state_memory, action, reward, next_state_memory, done):
        """n-step 메모리에 경험 추가 및 TD 오류 계산"""
        if np.all(state_memory[0] == 0) or np.all(state_memory[0] == 5):  # 시퀀스가 비어있는지 확인
            return
        
        # n-step 메모리에 추가
        self.n_step_memory.append(state_memory, next_state_memory, reward, done, action)
        n_step_sample = self.n_step_memory.sample()
        
        if n_step_sample is not None:
            state = n_step_sample['state'][0]
            next_state = n_step_sample['next_state'][-1]
            reward_list = n_step_sample['reward']
            done = n_step_sample['done'][-1]
            action = n_step_sample['action'][0]

            # n-step 리턴 계산
            n_step_return = np.sum([np.power(self.discount_factor, i) * r for i, r in enumerate(reward_list)])

            # TD 오류 계산을 위해 현재 Q값과 타깃 Q값 구하기
            state_tensor = tf.convert_to_tensor(np.expand_dims(state, axis=0), dtype=tf.float32)
            next_state_tensor = tf.convert_to_tensor(np.expand_dims(next_state, axis=0), dtype=tf.float32)
            
            # 현재 Q값
            _, current_q, _ = self.model(
                state_tensor, 
                self.train_num_quantile, 
                self.train_tau_min, 
                self.train_tau_max,
                training=True)
            current_q_value = current_q[0][action]
            
            # 다음 Q값 (Double DQN 방식)
            _, next_q, _ = self.model(
                next_state_tensor, 
                self.train_num_quantile, 
                self.train_tau_min, 
                self.train_tau_max,
                training=True)
            next_action = np.argmax(next_q[0])
            
            _, target_next_q, _ = self.target_model(
                next_state_tensor,
                self.train_num_quantile,
                self.train_tau_min,
                self.train_tau_max)
            target_value = target_next_q[0][next_action]
            
            # TD 타깃
            td_target = n_step_return + (1 - done) * self.discount_factor**self.n_step * target_value
            
            # TD 오류
            td_error = np.abs(td_target - current_q_value)
            
            # PER 메모리에 추가
            self.memory.add(td_error, (state, action, n_step_return, next_state, done))

    def train_model(self):
        """PER 메모리에서 샘플링하여 모델 업데이트"""
        if self.memory.tree.n_entries < self.batch_size:
            return 0.0
            
        # PER 메모리에서 배치 샘플링
        mini_batch, idxs, IS_weight = self.memory.sample(self.batch_size)
        
        # 배치 데이터 추출
        states = np.array([sample[0] for sample in mini_batch])
        actions = np.array([sample[1] for sample in mini_batch])
        rewards = np.array([sample[2] for sample in mini_batch])
        next_states = np.array([sample[3] for sample in mini_batch])
        dones = np.array([sample[4] for sample in mini_batch])

        # 변환
        states_tensor = tf.convert_to_tensor(states, dtype=tf.float32)
        next_states_tensor = tf.convert_to_tensor(next_states, dtype=tf.float32)
        rewards_tensor = tf.convert_to_tensor(rewards, dtype=tf.float32)
        actions_tensor = tf.convert_to_tensor(actions, dtype=tf.int32)
        dones_tensor = tf.convert_to_tensor(dones, dtype=tf.float32)
        IS_weight_tensor = tf.convert_to_tensor(IS_weight, dtype=tf.float32)
        
        # 학습 단계
        rainbow_variable = self.model.trainable_variables
        with tf.GradientTape() as tape:
            # 현재 상태의 분위수 예측 (학습 모드로 호출하여 노이즈 적용)
            theta_pred, _, sample = self.model(
                states_tensor, 
                self.train_num_quantile,
                self.train_tau_min, 
                self.train_tau_max,
                training=True)
            
            # 다음 상태의 분위수 예측 (Double DQN 적용)
            _, main_next_q, _ = self.model(
                next_states_tensor,
                self.train_num_quantile,
                self.train_tau_min,
                self.train_tau_max,
                training=True)
            
            theta_next, _, _ = self.target_model(
                next_states_tensor,
                self.train_num_quantile,
                self.train_tau_min,
                self.train_tau_max)
            
            # 다음 행동 선택 (Double DQN 방식)
            next_actions = tf.cast(tf.argmax(main_next_q, axis=1), tf.int32)
            
            # 선택된 행동에 대한 분위수 예측값 추출
            action_binary = tf.one_hot(actions_tensor, self.action_size)
            action_binary = tf.expand_dims(action_binary, axis=0)
            action_binary = tf.tile(action_binary, [self.train_num_quantile, 1, 1])
            
            theta_pred_action = tf.reduce_sum(theta_pred * action_binary, axis=2)
            
            # 타깃 분위수 계산
            target_actions_one_hot = tf.one_hot(next_actions, self.action_size)
            target_actions_one_hot = tf.expand_dims(target_actions_one_hot, axis=0)
            target_actions_one_hot = tf.tile(target_actions_one_hot, [self.train_num_quantile, 1, 1])
            
            theta_next_action = tf.reduce_sum(theta_next * target_actions_one_hot, axis=2)
            theta_target = rewards_tensor + (1.0 - dones_tensor) * (self.discount_factor ** self.n_step) * theta_next_action
            
            # Huber 손실 계산
            theta_target = tf.stop_gradient(theta_target)
            
            # Shape 호환성 확인을 위한 디버깅 출력
            # print("theta_pred_action shape:", theta_pred_action.shape)
            # print("theta_target shape:", theta_target.shape)
            
            # 두 텐서가 이미 같은 shape (num_quantile, batch_size)라면
            # 직접 오차 계산 가능
            error_loss = theta_target - theta_pred_action  # [num_quantile, batch_size]
            
            # Huber 손실 적용
            huber_threshold = tf.constant(self.huber_threshold, dtype=tf.float32)
            huber_loss = tf.where(
                tf.abs(error_loss) <= huber_threshold,
                0.5 * tf.square(error_loss),
                huber_threshold * (tf.abs(error_loss) - 0.5 * huber_threshold)
            )  # [num_quantile, batch_size]
            
            # 분위수 값 재구성
            sample_reshaped = tf.reshape(sample, [self.train_num_quantile, self.batch_size])  # [num_quantile, batch_size]
            tau = sample_reshaped  # [num_quantile, batch_size]
            inv_tau = 1.0 - tau  # [num_quantile, batch_size]
            
            # 분위수 회귀 손실 계산
            quantile_loss = tf.where(error_loss < 0, inv_tau * huber_loss, tau * huber_loss)  # [num_quantile, batch_size]
            
            # 중요도 샘플링 가중치 적용
            # IS_weight_tensor는 [batch_size] 형태
            IS_weight_transposed = tf.tile(tf.expand_dims(IS_weight_tensor, axis=0), [self.train_num_quantile, 1])  # [num_quantile, batch_size]
            loss = quantile_loss * IS_weight_transposed  # [num_quantile, batch_size]
            loss = tf.reduce_mean(tf.reduce_sum(loss, axis=0))  # 스칼라
            
        # 그래디언트 계산 및 적용
        grads = tape.gradient(loss, rainbow_variable)
        self.optimizer.apply_gradients(zip(grads, rainbow_variable))
        
        # 타깃 네트워크 주기적 업데이트
        self.train_step_counter += 1
        if self.train_step_counter % self.target_update_freq == 0:
            self.update_target_model()
            print(f"Target network updated at step {self.train_step_counter}")
        
        # PER 메모리 우선순위 업데이트
        _, current_q, _ = self.model(
            states_tensor,
            self.train_num_quantile,
            self.train_tau_min,
            self.train_tau_max)
        
        current_q_batch = np.array(current_q)
        current_q_values = np.array([q_vals[a] for q_vals, a in zip(current_q_batch, actions)])
        
        _, target_q, _ = self.target_model(
            next_states_tensor,
            self.train_num_quantile,
            self.train_tau_min,
            self.train_tau_max)
        
        target_q_batch = np.array(target_q)
        next_actions_np = np.array(next_actions)
        target_q_values = np.array([q_vals[a] for q_vals, a in zip(target_q_batch, next_actions_np)])
        
        td_targets = rewards + (1 - dones) * (self.discount_factor ** self.n_step) * target_q_values
        td_errors = np.abs(td_targets - current_q_values)
        
        # PER 메모리 업데이트
        for i in range(self.batch_size):
            idx = idxs[i]
            self.memory.update(idx, td_errors[i])
            
        return loss.numpy()
        
    def end_episode(self, episode_reward=None):
        """에피소드 종료 시 호출하는 메서드"""
        self.episode_counter += 1
        
        # Noisy Network 사용 시 epsilon 감소 필요 없음 (내재적 탐색)
        if not self.use_noisy_net:
            # 10 에피소드마다 입실론 감소
            if self.episode_counter % self.epsilon_update_frequency == 0 and self.epsilon > self.epsilon_min:
                old_epsilon = self.epsilon
                self.epsilon *= self.epsilon_decay
                print(f"Epsilon decreased from {old_epsilon:.4f} to {self.epsilon:.4f}")
        
        # 현재 입실론 값 반환 (TensorBoard 로깅용)
        return self.epsilon