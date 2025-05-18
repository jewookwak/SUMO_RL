# train/lstm_dqn_nstep_agent.py
import tensorflow as tf
import numpy as np
from collections import deque
import random
from tensorflow.keras.optimizers import Adam
from train.network.LSTM_networks import LSTMDQN
from train.replay_buffer import ReplayBuffer, NStepMemory

class LSTMDQNNStepAgent:
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
        
        # 에피소드 카운터와 입실론 감소 주기 설정
        self.episode_counter = 0
        self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소

        # 모델과 타깃 모델 생성
        self.model = LSTMDQN(state_size, action_size, self.batch_size, self.sequence_length)
        self.target_model = LSTMDQN(state_size, action_size, self.batch_size, self.sequence_length)
        self.optimizer = Adam(learning_rate=self.learning_rate)

        # n-step 메모리와 리플레이 버퍼 초기화
        self.n_step_memory = NStepMemory(maxlen=self.n_step)
        self.memory = ReplayBuffer(config.MEMORY_SIZE)

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
        """n-step 메모리에 경험 추가 및 n-step 리턴 계산"""
        if np.all(state_memory[0] == 0):  # 시퀀스가 비어있는지 확인
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

            # 메모리에 추가
            self.memory.add_buffer(state, action, n_step_return, next_state, done)

    def train_model(self):
        """메모리에서 샘플링하여 모델 업데이트"""
        if len(self.memory.buffer) < self.batch_size:
            return 0.0
        
        # 리플레이 버퍼에서 미니배치 샘플링
        states_raw, actions, rewards, next_states_raw, dones = self.memory.sample_batch(self.batch_size)
        
        # 데이터 차원 확인 및 조정
        # LSTM 네트워크가 기대하는 정확한 형태로 변환
        states = self._prepare_batch_for_lstm(states_raw)
        next_states = self._prepare_batch_for_lstm(next_states_raw)
        
        # 학습 단계
        dqn_variable = self.model.trainable_variables
        with tf.GradientTape() as tape:
            tape.watch(dqn_variable)
            
            # 현재 상태에 대한 Q값
            predicts = self.model(states)
            one_hot_action = tf.one_hot(actions, self.action_size)
            predicts = tf.reduce_sum(one_hot_action * predicts, axis=1)
            
            # 다음 상태에 대한 Q값 (Double DQN)
            next_q = self.model(next_states)
            next_actions = tf.argmax(next_q, axis=1)
            
            target_q = self.target_model(next_states)
            target_value = tf.reduce_sum(
                tf.one_hot(next_actions, self.action_size) * target_q, axis=1
            )
            
            # n-step 리턴이 이미 계산되어 있으므로 discount_factor는 1로 설정
            targets = rewards + (1-dones) * self.discount_factor**self.n_step * target_value
            
            # 손실 계산
            loss = tf.reduce_mean(tf.square(targets - predicts))
            
        # 그래디언트 계산 및 적용
        dqn_grads = tape.gradient(loss, dqn_variable)
        self.optimizer.apply_gradients(zip(dqn_grads, dqn_variable))
        
        return loss.numpy()
    
    def _prepare_batch_for_lstm(self, states):
        """
        상태 데이터를 LSTM 네트워크에 맞는 형식으로 준비
        LSTM_networks.py의 입력 요구사항에 맞게 조정
        """
        # 상태가 이미 올바른 형태(batch_size, sequence_length, state_size)인지 확인
        states = np.array(states)
        
        # 차원이 부족한 경우 처리
        if len(states.shape) == 2:  # (batch_size, state_size)
            # 시퀀스 차원 추가 (batch_size, sequence_length, state_size)
            padded_states = np.zeros((self.batch_size, self.sequence_length, self.state_size))
            
            # 각 배치 항목에 대해 상태를 시퀀스의 마지막 항목으로 설정
            for i, state in enumerate(states):
                # 일부 모델은 전체 시퀀스가 필요할 수 있으므로 
                # 마지막 타임스텝에만 실제 상태를 넣고 나머지는 0으로 패딩
                padded_states[i, -1, :] = state
            
            return padded_states
        
        # 이미 올바른 형태인 경우
        elif len(states.shape) == 3:
            # 필요한 경우 배치 크기와 시퀀스 길이를 조정
            if states.shape[0] != self.batch_size or states.shape[1] != self.sequence_length:
                padded_states = np.zeros((self.batch_size, self.sequence_length, self.state_size))
                
                # 실제 데이터 복사
                min_batch = min(states.shape[0], self.batch_size)
                min_seq = min(states.shape[1], self.sequence_length)
                
                for i in range(min_batch):
                    padded_states[i, -min_seq:, :] = states[i, -min_seq:, :]
                
                return padded_states
            else:
                return states
                
        # 다른 형태인 경우 - 오류 출력
        else:
            print(f"Unexpected state shape: {states.shape}, expected (batch_size, state_size) or (batch_size, sequence_length, state_size)")
            # 기본값으로 0으로 채워진 적절한 크기의 배열 반환
            return np.zeros((self.batch_size, self.sequence_length, self.state_size))
        
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