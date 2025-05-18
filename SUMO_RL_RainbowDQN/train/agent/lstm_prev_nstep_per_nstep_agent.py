# train/lstm_prev_nstep_per_nstep_agent.py
import tensorflow as tf
import numpy as np
from collections import deque
import random
from tensorflow.keras.optimizers import Adam
from train.network.LSTM_networks import LSTMDQN
from train.replay_buffer import PrioritizedReplayBuffer, NStepMemory

class LSTMPrevNStepPERNStepAgent:
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

        # 모델과 타깃 모델 생성
        self.model = LSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
        self.target_model = LSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
        self.optimizer = Adam(learning_rate=self.config.LEARNING_RATE)
        
        # n-step 메모리와 PER 버퍼 초기화
        self.n_step_memory = NStepMemory(maxlen=self.n_step)
        self.memory = PrioritizedReplayBuffer(capacity=config.MEMORY_SIZE)

        # 타깃 모델 초기화
        self.update_target_model()

    def update_target_model(self):
        """타깃 모델을 모델의 가중치로 업데이트"""
        self.target_model.set_weights(self.model.get_weights())

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
        """PER 메모리에서 샘플링하여 모델 업데이트"""
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
            
        # 그래디언트 계산 및 적용
        dqn_grads = tape.gradient(error, dqn_variable)
        self.optimizer.apply_gradients(zip(dqn_grads, dqn_variable))
        
        # PER 메모리 우선순위 업데이트
        state_value = np.array(self.model(tf.convert_to_tensor(states, dtype=tf.float32)))
        state_value = np.array([sv[a] for a, sv in zip(actions, state_value)])
        
        td_error = np.abs(target_value.numpy() - state_value)
        
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
        
        # n-step 메모리 초기화
        self.n_step_memory.clear()
        
        # 현재 입실론 값 반환 (TensorBoard 로깅용)
        return self.epsilon