# train/lstm_dqn_per_agent.py
import tensorflow as tf
import numpy as np
from tensorflow.keras.optimizers import Adam
from train.network.LSTM_networks import LSTMDQN
from train.replay_buffer import PrioritizedReplayBuffer

class LSTMDQNPERAgent:
    def __init__(self, state_size, action_size, config):
        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size
        self.config = config
        self.sequence_length = config.SEQUENCE_LENGTH
        self.batch_size = config.BATCH_SIZE

        # 하이퍼파라미터 설정
        self.discount_factor = config.DISCOUNT_FACTOR
        self.learning_rate = config.LEARNING_RATE
        self.epsilon = config.EPSILON
        self.epsilon_decay = config.EPSILON_DECAY
        self.epsilon_min = config.EPSILON_MIN
        
        # 에피소드 카운터와 입실론 감소 주기 설정
        self.episode_counter = 0
        self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소

        # 모델과 타깃 모델 생성
        self.model = LSTMDQN(state_size, action_size, self.batch_size, self.sequence_length)
        self.target_model = LSTMDQN(state_size, action_size, self.batch_size, self.sequence_length)
        self.optimizer = Adam(learning_rate=self.learning_rate)

        # PER 버퍼만 사용 (다른 메모리 버퍼 제거)
        self.memory = PrioritizedReplayBuffer(capacity=config.MEMORY_SIZE)
        
        # 학습 카운터
        self.train_step_counter = 0

        # 타깃 모델 초기화
        self.update_target_model()

    def update_target_model(self):
        """타깃 모델을 모델의 가중치로 업데이트"""
        self.target_model.set_weights(self.model.get_weights())

    def get_action(self, state_memory):
        """입실론 탐욕 정책으로 행동 선택"""
        if np.all(state_memory[0] == 0) or np.all(state_memory[0] == 5):  # 시퀀스가 비어있는지 확인
            return 5
        
        
        state = np.expand_dims(state_memory, axis=0)
        if np.random.rand() <= self.epsilon:
            return np.random.randint(self.action_size)
        else:
            q_value = self.model(state)
            return np.argmax(q_value[0])

    def append_sample(self, state_memory, action, reward, next_state_memory, done):
        """경험을 PER 메모리에 추가"""
        if np.all(state_memory[0] == 5):  # 시퀀스가 비어있는지 확인
            return
        
        # TD 오류 계산을 위해 현재 Q값과 타깃 Q값 구하기
        state_tensor = tf.convert_to_tensor(np.expand_dims(state_memory, axis=0), dtype=tf.float32)
        next_state_tensor = tf.convert_to_tensor(np.expand_dims(next_state_memory, axis=0), dtype=tf.float32)
        
        # 현재 Q값
        current_q = self.model(state_tensor)[0]
        current_q_value = current_q[action]
        
        # 다음 Q값 (Double DQN 방식)
        next_q = self.model(next_state_tensor)[0]
        next_action = np.argmax(next_q)
        
        target_next_q = self.target_model(next_state_tensor)[0]
        target_value = target_next_q[next_action]
        
        # TD 타깃
        td_target = reward + (1 - done) * self.discount_factor * target_value
        
        # TD 오류
        td_error = np.abs(td_target - current_q_value)
        
        # PER 메모리에 추가 (전체 state_memory와 next_state_memory를 저장)
        self.memory.add(td_error, (state_memory, action, reward, next_state_memory, done))

    def train_model(self):
        """PER 메모리에서 샘플링하여 모델 업데이트"""
        if self.memory.tree.n_entries < self.batch_size:
            return 0.0
            
        # 입실론 감소 로직 제거 (end_episode에서 처리)
            
        # PER 메모리에서 배치 샘플링
        mini_batch, idxs, IS_weight = self.memory.sample(self.batch_size)
        
        # 배치 데이터 추출 (state_memory와 next_state_memory는 이미 시퀀스 형태)
        states = np.array([sample[0] for sample in mini_batch])
        actions = np.array([sample[1] for sample in mini_batch])
        rewards = np.array([sample[2] for sample in mini_batch])
        next_states = np.array([sample[3] for sample in mini_batch])
        dones = np.array([sample[4] for sample in mini_batch])

        # 학습 단계
        dqn_variable = self.model.trainable_variables
        with tf.GradientTape() as tape:
            tape.watch(dqn_variable)
            
            # 현재 상태에 대한 Q값
            rewards = tf.convert_to_tensor(rewards, dtype=tf.float32)
            actions = tf.convert_to_tensor(actions, dtype=tf.int32)
            dones = tf.convert_to_tensor(dones, dtype=tf.float32)
            
            # 다음 상태에 대한 Q값
            target_q = self.target_model(tf.convert_to_tensor(next_states, dtype=tf.float32))
            main_q = self.model(tf.convert_to_tensor(next_states, dtype=tf.float32))
            next_action = tf.argmax(main_q, axis=1)
            target_value = tf.reduce_sum(tf.one_hot(next_action, self.action_size) * target_q, axis=1)
            
            # 타깃 계산
            target_value = (1-dones) * self.discount_factor * target_value + rewards
            
            # 현재 모델의 Q값 예측
            main_q = self.model(tf.convert_to_tensor(states, dtype=tf.float32))
            main_value = tf.reduce_sum(tf.one_hot(actions, self.action_size) * main_q, axis=1)
            
            # TD 오류 및 손실 계산 (IS 가중치 적용)
            error = tf.square(target_value - main_value)
            error = error * tf.convert_to_tensor(IS_weight, dtype=tf.float32)
            error = tf.reduce_mean(error)
            
        # 그래디언트 계산 및 적용
        dqn_grads = tape.gradient(error, dqn_variable)
        self.optimizer.apply_gradients(zip(dqn_grads, dqn_variable))
        
        # PER 메모리 우선순위 업데이트
        q_values = np.array(self.model(tf.convert_to_tensor(states, dtype=tf.float32)))
        q_values = np.array([q[a] for a, q in zip(actions, q_values)])
        
        td_error = np.abs(target_value.numpy() - q_values)
        
        for i in range(self.batch_size):
            idx = idxs[i]
            self.memory.update(idx, td_error[i])
            
        return error.numpy()
        
    def end_episode(self, episode_reward=None):
        """에피소드 종료 시 호출하는 메서드"""
        self.episode_counter += 1
        
        # 10 에피소드마다 입실론 감소
        if self.episode_counter % self.epsilon_update_frequency == 0 and self.epsilon > self.epsilon_min:
            old_epsilon = self.epsilon
            self.epsilon *= self.epsilon_decay
            print(f"Epsilon decreased from {old_epsilon:.4f} to {self.epsilon:.4f}")
        
        # 현재 입실론 값 반환 (TensorBoard 로깅용)
        return self.epsilon