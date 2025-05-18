# train/dqn_iqn_agent.py
import tensorflow as tf
import numpy as np
from collections import deque
import random
from tensorflow.keras.optimizers import Adam
from train.network.networks import DQNIQN 

class DQNIQNAgent:
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
        
        # IQN 하이퍼파라미터
        self.embedding_dim = config.IQN_EMBEDDING_DIM
        self.get_action_num_quantile = config.IQN_ACTION_QUANTILES
        self.get_action_tau_min = config.IQN_ACTION_TAU_MIN
        self.get_action_tau_max = config.IQN_ACTION_TAU_MAX
        self.train_num_quantile = config.IQN_TRAIN_QUANTILES
        self.train_tau_min = config.IQN_TRAIN_TAU_MIN
        self.train_tau_max = config.IQN_TRAIN_TAU_MAX
        self.huber_threshold = config.IQN_HUBER_THRESHOLD
        
        # 에피소드 카운터와 입실론 감소 주기 설정
        self.episode_counter = 0
        self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소

        # 모델과 타깃 모델 생성
        self.model = DQNIQN(self.state_size, self.action_size, self.embedding_dim)
        self.target_model = DQNIQN(self.state_size, self.action_size, self.embedding_dim)
        self.optimizer = Adam(learning_rate=config.LEARNING_RATE)

        # 일반 메모리 버퍼 초기화 (PER 아님)
        self.memory = deque(maxlen=config.MEMORY_SIZE)
        
        # 타깃 모델 초기화
        self.update_target_model()

    def update_target_model(self):
        """타깃 모델을 모델의 가중치로 업데이트"""
        self.target_model.set_weights(self.model.get_weights())

    def get_action(self, state):
        """입실론 탐욕 정책으로 행동 선택"""
        state = np.reshape(state, [1, self.state_size])
        if np.random.rand() <= self.epsilon:
            return np.random.randint(self.action_size)
        else:
            _, q_value, _ = self.model(
                tf.convert_to_tensor(state, dtype=tf.float32),
                self.get_action_num_quantile,
                self.get_action_tau_min,
                self.get_action_tau_max)
            return np.argmax(q_value[0])

    def append_sample(self, state, action, reward, next_state, done):
        """일반 리플레이 메모리에 경험 추가 (n-step 없음)"""
        self.memory.append((state, action, reward, next_state, done))

    def train_model(self):
        """메모리에서 미니배치를 샘플링하여 모델 업데이트"""
        if len(self.memory) < self.batch_size:
            return 0.0
            
        # 일반 리플레이 메모리에서 배치 샘플링 (PER 없음)
        mini_batch = random.sample(self.memory, self.batch_size)
        
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
        
        # 학습 단계
        iqn_variable = self.model.trainable_variables
        with tf.GradientTape() as tape:
            # 현재 상태의 분위수 예측
            theta_pred, _, sample = self.model(
                states_tensor, 
                self.train_num_quantile,
                self.train_tau_min, 
                self.train_tau_max)
            
            # 다음 상태의 분위수 예측 (메인 모델과 타깃 모델)
            _, main_next_q, _ = self.model(
                next_states_tensor,
                self.train_num_quantile,
                self.train_tau_min,
                self.train_tau_max)
            
            theta_next, _, _ = self.target_model(
                next_states_tensor,
                self.train_num_quantile,
                self.train_tau_min,
                self.train_tau_max)
            
            # 다음 행동 선택 (Double DQN 방식)
            next_actions = tf.argmax(main_next_q, axis=1)
            
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
            theta_target = rewards_tensor + (1.0 - dones_tensor) * self.discount_factor * theta_next_action
            
            # Huber 손실 계산
            theta_target = tf.stop_gradient(theta_target)
            theta_target_tile = tf.expand_dims(theta_target, axis=0)
            theta_target_tile = tf.tile(theta_target_tile, [self.train_num_quantile, 1, 1])
            
            theta_pred_tile = tf.expand_dims(theta_pred_action, axis=2)
            theta_pred_tile = tf.tile(theta_pred_tile, [1, 1, self.train_num_quantile])
            
            # 오차 계산
            error_loss = theta_target_tile - theta_pred_tile
            
            # Huber 손실 적용
            huber_loss = tf.where(
                tf.abs(error_loss) <= self.huber_threshold,
                0.5 * tf.square(error_loss),
                self.huber_threshold * (tf.abs(error_loss) - 0.5 * self.huber_threshold)
            )
            
            # 분위수 회귀 손실 계산
            tau = tf.reshape(sample, [self.train_num_quantile, self.batch_size, 1])
            tau = tf.tile(tau, [1, 1, self.train_num_quantile])
            inv_tau = 1.0 - tau
            
            quantile_loss = tf.where(error_loss < 0, inv_tau * huber_loss, tau * huber_loss)
            quantile_loss = tf.reduce_mean(quantile_loss, axis=2)
            
            # 손실 계산 (중요도 샘플링 가중치 없음)
            loss = tf.reduce_mean(tf.reduce_sum(quantile_loss, axis=0))
            
        # 그래디언트 계산 및 적용
        grads = tape.gradient(loss, iqn_variable)
        self.optimizer.apply_gradients(zip(grads, iqn_variable))
            
        return loss.numpy()
        
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