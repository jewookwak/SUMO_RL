# train/dqn_nstep_agent.py
import tensorflow as tf
import numpy as np
from collections import deque
from tensorflow.keras.optimizers import Adam
from train.network.networks import DQN
from train.replay_buffer import ReplayBuffer, NStepMemory

class DQNNStepAgent:
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
        
        # 에피소드 카운터와 입실론 감소 주기 설정
        self.episode_counter = 0
        self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소

        # 모델과 타깃 모델 생성
        self.model = DQN(state_size, action_size)
        self.target_model = DQN(state_size, action_size)
        self.optimizer = Adam(learning_rate=self.learning_rate)

        # n-step 메모리와 일반 버퍼 초기화
        self.n_step_memory = NStepMemory(maxlen=self.n_step)
        self.buffer = ReplayBuffer(config.MEMORY_SIZE)

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
            q_value = self.model(state)
            return np.argmax(q_value[0])

    def append_sample(self, state, action, reward, next_state, done):
        """n-step 메모리에 경험 추가 및 n-step 리턴 계산"""
        self.n_step_memory.append(state, next_state, reward, done, action)
        n_step_sample = self.n_step_memory.sample()
        
        if n_step_sample is not None:
            state = n_step_sample['state'][0]
            next_state = n_step_sample['next_state'][-1]
            reward_list = n_step_sample['reward']
            done = n_step_sample['done'][-1]
            action = n_step_sample['action'][0]

            # n-step 리턴 계산
            n_step_return = np.sum([np.power(self.discount_factor, i) * r for i, r in enumerate(reward_list)])

            # 일반 버퍼에 n-step 리턴 추가
            self.buffer.add_buffer(
                np.reshape(state, [1, self.state_size]), 
                action, 
                n_step_return,
                np.reshape(next_state, [1, self.state_size]), 
                done
            )

    def train_model(self):
        """버퍼에서 샘플링하여 모델 업데이트"""
        if self.buffer.buffer_count() < self.batch_size:
            return 0.0

        # 입실론 감소는 train_model에서 제거 (end_episode에서 처리)
            
        # 리플레이 버퍼에서 미니배치 샘플링
        states, actions, rewards, next_states, dones = self.buffer.sample_batch(self.batch_size)
        
        # 학습 단계
        dqn_variable = self.model.trainable_variables
        with tf.GradientTape() as tape:
            tape.watch(dqn_variable)
            
            # 현재 상태에 대한 Q값
            predicts = self.model(states)
            one_hot_action = tf.one_hot(actions, self.action_size)
            predicts = tf.reduce_sum(one_hot_action * predicts, axis=1)

            # 다음 상태에 대한 Q값 (Double DQN)
            target_predicts = self.target_model(next_states)
            next_actions = tf.argmax(self.model(next_states), axis=1)
            target_predicts = tf.reduce_sum(
                tf.one_hot(next_actions, self.action_size) * target_predicts, axis=1
            )
            
            # n-step 타깃 계산 (미니배치의 모든 경험이 n-step이므로 n-step 승수 제외)
            targets = rewards + (1-dones) * self.discount_factor * target_predicts
            
            # 손실 계산
            loss = tf.reduce_mean(tf.square(targets - predicts))
            
        # 그래디언트 계산 및 적용
        dqn_grads = tape.gradient(loss, dqn_variable)
        self.optimizer.apply_gradients(zip(dqn_grads, dqn_variable))
        
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