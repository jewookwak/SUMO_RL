# train/LSTM_autoencoder_trainer.py
import os
import numpy as np
import tensorflow as tf
# from tensorflow.keras.optimizers import Adam
from tensorflow.keras.optimizers.legacy import Adam

from datetime import datetime
from collections import deque
import random
import matplotlib.pyplot as plt
import time
from train.network.LSTM_autoencoder_network import EnhancedLSTMDQN, LSTMAutoencoder
from train.replay_buffer import ReplayBuffer

class PREVNSTEPLSTMAutoencoderTrainer:
    def __init__(self, env, config, agent=None,agent_name=None):
        self.env = env
        self.config = config
        self.train_step_counter = 0
        
        # Create save directories
        os.makedirs(self.config.WEIGHTS_PATH, exist_ok=True)
        os.makedirs(self.config.GRAPH_PATH, exist_ok=True)
        os.makedirs(self.config.LOG_PATH, exist_ok=True)
        
        # Initialize model parameters
        self.state_size = len(env.observation_space)
        self.action_size = len(env.action_space)
        self.sequence_length = config.PREV_SEQUENCE_LENGTH
        self.batch_size = self.config.BATCH_SIZE
        
        # 초기화를 위한 변수들
        self.best_reward = float('-inf')
        self.avg_reward = 0
        self.episode_counter = 0
        self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소
        


        # 에이전트 초기화 방식 결정
        if agent is not None:
            self.agent = agent
            self.agent_name = agent_name
            self.model = agent.model
            self.target_model = agent.target_model
            self.optimizer = agent.optimizer if hasattr(agent, 'optimizer') else Adam(learning_rate=self.config.LEARNING_RATE)
            self.epsilon = agent.epsilon if hasattr(agent, 'epsilon') else self.config.EPSILON
            self.epsilon_decay = agent.epsilon_decay if hasattr(agent, 'epsilon_decay') else self.config.EPSILON_DECAY
            self.epsilon_min = agent.epsilon_min if hasattr(agent, 'epsilon_min') else self.config.EPSILON_MIN
            self.discount_factor = agent.discount_factor if hasattr(agent, 'discount_factor') else self.config.DISCOUNT_FACTOR
            self.memory = agent.memory if hasattr(agent, 'memory') else deque(maxlen=config.MEMORY_SIZE)
            self.using_external_agent = True
        else:
            # 새로운 LSTM 오토인코더 모델 초기화
            self.model = EnhancedLSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
            self.target_model = EnhancedLSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
            self.optimizer = Adam(learning_rate=self.config.LEARNING_RATE)
            
            # 재구성 손실을 위한 별도의 옵티마이저
            self.recon_optimizer = Adam(learning_rate=self.config.LEARNING_RATE)
            
            # 하이퍼파라미터 설정
            self.epsilon = self.config.EPSILON
            self.epsilon_decay = self.config.EPSILON_DECAY
            self.epsilon_min = self.config.EPSILON_MIN
            self.discount_factor = self.config.DISCOUNT_FACTOR
            
            # Memory buffers for experience replay
            self.memory = deque(maxlen=5000*6)
            self.memory0 = deque(maxlen=5000)  # Action 0 memory
            self.memory1 = deque(maxlen=5000)  # Action 1 memory
            self.memory2 = deque(maxlen=5000)  # Action 2 memory
            self.memory3 = deque(maxlen=5000)  # Action 3 memory
            self.memory4 = deque(maxlen=5000)  # Action 4 memory
            self.memory5 = deque(maxlen=5000)  # Action 5 memory
            
            self.using_external_agent = False
            
            # 초기화 단계: 오토인코더 사전 훈련 여부 플래그
            self.is_autoencoder_pretrained = False
        
        # TensorBoard setup
        self._setup_tensorboard()

        # Initialize target network
        self.update_target_network(initial_update = True)

    def update_target_network(self,initial_update):
        """Update target network with weights from main network"""
        if self.using_external_agent:
            self.agent.update_target_model()
        else:
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
                        tgt_encoder = getattr(self.target_model, tgt_encoder_name)
                        
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
                print('initial update is ',initial_update)
                print(f"레이어별 가중치 복사 실패: {e}")
                print("전체 모델 가중치 복사 시도는 생략합니다.")
    def _debug_model_layers(self):
        """모델과 타겟 모델의 레이어 구조 비교"""
        print("\n===== 모델 레이어 디버그 =====")
        
        print("원본 모델 레이어:")
        for i, layer in enumerate(self.model.layers):
            print(f"  {i}: {layer.name} ({layer.__class__.__name__})")
        
        print("\n타겟 모델 레이어:")
        for i, layer in enumerate(self.target_model.layers):
            print(f"  {i}: {layer.name} ({layer.__class__.__name__})")
        
        # 오토인코더 레이어 비교
        for encoder_name in ['vehicle_autoencoder', 'ego_autoencoder', 'space_autoencoder']:
            print(f"\n원본 모델 {encoder_name} 레이어:")
            src_encoder = getattr(self.model, encoder_name)
            for i, layer in enumerate(src_encoder.layers):
                print(f"  {i}: {layer.name} ({layer.__class__.__name__})")
            
            print(f"타겟 모델 {encoder_name} 레이어:")
            tgt_encoder = getattr(self.target_model, encoder_name)
            for i, layer in enumerate(tgt_encoder.layers):
                print(f"  {i}: {layer.name} ({layer.__class__.__name__})")        
    def _log_hyperparameters(self):
        """로그에 하이퍼파라미터 기록"""
        with self.summary_writer.as_default():
            # 하이퍼파라미터 텍스트로 로깅
            hyperparams = (
                f"Learning Rate: {self.config.LEARNING_RATE}\n"
                f"Epsilon Start: {self.config.EPSILON}\n"
                f"Epsilon Decay: {self.config.EPSILON_DECAY}\n"
                f"Epsilon Min: {self.config.EPSILON_MIN}\n"
                f"Discount Factor: {self.config.DISCOUNT_FACTOR}\n"
                f"Batch Size: {self.config.BATCH_SIZE}\n"
                f"Sequence Length: {self.config.SEQUENCE_LENGTH}\n"
                f"Memory Size: {self.config.MEMORY_SIZE}\n"
                f"Using External Agent: {self.using_external_agent}\n"
                f"Using Autoencoder: True\n"
            )
            tf.summary.text('hyperparameters', hyperparams, step=0)

    def get_action(self, state_memory):
        """Choose action using epsilon-greedy policy"""
        if np.all(state_memory[0] == 5):  # Check if state memory has data
            return 5
        else:
            state = np.expand_dims(state_memory, axis=0)
            
            if np.random.rand() <= self.epsilon:
                return np.random.randint(self.action_size)
            else:
                q_value = self.model(state)
                action = np.argmax(q_value[0])
                return action

    def append_sample(self, state, action, reward, next_state, done):
        """Add experience to the appropriate memory buffer based on action"""
        if action == 0:
            self.memory0.append((state, action, reward, next_state, done))
        elif action == 1:
            self.memory1.append((state, action, reward, next_state, done))
        elif action == 2:
            self.memory2.append((state, action, reward, next_state, done))
        elif action == 3:
            self.memory3.append((state, action, reward, next_state, done))
        elif action == 4:
            self.memory4.append((state, action, reward, next_state, done))
        elif action == 5:
            self.memory5.append((state, action, reward, next_state, done))

    def consolidate_memory(self):
        """Combine action-specific memories into the main memory buffer"""
        memory_limit = 10  # Minimum samples per action buffer
        
        if (len(self.memory0) >= memory_limit and 
            len(self.memory1) >= memory_limit and 
            len(self.memory2) >= memory_limit and 
            len(self.memory3) >= memory_limit and 
            len(self.memory4) >= memory_limit and 
            len(self.memory5) >= memory_limit):
            
            self.memory.extend(self.memory0)
            self.memory.extend(self.memory1)
            self.memory.extend(self.memory2)
            self.memory.extend(self.memory3)
            self.memory.extend(self.memory4)
            self.memory.extend(self.memory5)
            
            # 메모리 버퍼 초기화 추가 (중복 데이터 방지)
            self.memory0.clear()
            self.memory1.clear()
            self.memory2.clear()
            self.memory3.clear()
            self.memory4.clear()
            self.memory5.clear()

    def pretrain_autoencoders(self, train_data, epochs=50):
        """
        오토인코더 부분을 사전 훈련하는 함수
        """
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
            
            avg_vehicle_loss = total_vehicle_loss / (len(vehicle_data) * num_batches)
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
            
            avg_ego_loss = total_ego_loss / num_batches
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
            
            avg_space_loss = total_space_loss / (len(space_data) * num_batches)
            history['space_loss'].append(avg_space_loss)
            
            # 진행 상황 출력
            elapsed_time = time.time() - start_time
            print(f"Epoch {epoch+1}/{epochs}, Time: {elapsed_time:.2f}s")
            print(f"  Vehicle Loss: {avg_vehicle_loss:.6f}")
            print(f"  Ego Loss: {avg_ego_loss:.6f}")
            print(f"  Space Loss: {avg_space_loss:.6f}")
            
            # 10 에포크마다 모델 저장
            if (epoch + 1) % 10 == 0:                
                # 오토인코더 가중치 저장
                self._save_autoencoder_weights(epoch)
            
            # TensorBoard에 손실 로깅
            with self.summary_writer.as_default():
                # 기본 손실 값 로깅
                tf.summary.scalar('autoencoder/vehicle_loss', avg_vehicle_loss, step=epoch)
                tf.summary.scalar('autoencoder/ego_loss', avg_ego_loss, step=epoch)
                tf.summary.scalar('autoencoder/space_loss', avg_space_loss, step=epoch)
                
                # 총 손실 계산 및 로깅
                total_loss = avg_vehicle_loss + avg_ego_loss + avg_space_loss
                tf.summary.scalar('autoencoder/total_loss', total_loss, step=epoch)
                
                # 매 5 에포크마다 가중치 히스토그램 로깅
                if epoch % 5 == 0:
                    # 오토인코더 가중치 히스토그램 로깅
                    for encoder_name, encoder in [
                        ('vehicle', self.model.vehicle_autoencoder),
                        ('ego', self.model.ego_autoencoder),
                        ('space', self.model.space_autoencoder)
                    ]:
                        for layer in encoder.layers:
                            for weight in layer.weights:
                                tf.summary.histogram(
                                    f"autoencoder/{encoder_name}/{layer.name}/{weight.name}",
                                    weight,
                                    step=epoch
                                )
                
                # 재구성 이미지 샘플 로깅 (에포크마다 첫 번째 배치의 첫 번째 샘플)
                if epoch % 10 == 0 and len(vehicle_data[0]) > 0:
                    # 차량 상태 재구성 예시
                    sample_input = vehicle_data[0][0:1]  # 첫 번째 배치의 첫 번째 샘플
                    reconstructed, _ = self.model.vehicle_autoencoder(sample_input)
                    
                    # 입력과 출력 데이터를 텍스트로 로깅 (이미지로 변환하기 어려운 경우)
                    # sample_input이 텐서인지 NumPy 배열인지 확인하고 처리
                    if isinstance(sample_input, tf.Tensor):
                        input_data = sample_input.numpy()[0]
                    else:
                        input_data = sample_input[0]  # 이미 NumPy 배열인 경우

                    # reconstructed 결과도 같은 방식으로 처리
                    if isinstance(reconstructed, tf.Tensor):
                        output_data = reconstructed.numpy()[0]
                    else:
                        output_data = reconstructed[0]  # 이미 NumPy 배열인 경우

                    tf.summary.text(
                        f'autoencoder/vehicle_reconstruction_epoch_{epoch}',
                        f"Input: {input_data}\nOutput: {output_data}",
                        step=epoch
                    )
        
        print("===== 오토인코더 사전 훈련 완료 =====")
        
        # 훈련 완료 후 최종 결과 로깅
        with self.summary_writer.as_default():
            # 에포크별 손실 추세 상세 로깅
            for e, (v_loss, e_loss, s_loss) in enumerate(zip(
                    history['vehicle_loss'], 
                    history['ego_loss'], 
                    history['space_loss'])):
                
                # 손실 값 로깅 (훈련 과정에서 이미 로깅했지만, 기록 보존을 위해 다시 로깅)
                tf.summary.scalar('autoencoder_history/vehicle_loss', v_loss, step=e)
                tf.summary.scalar('autoencoder_history/ego_loss', e_loss, step=e)
                tf.summary.scalar('autoencoder_history/space_loss', s_loss, step=e)
            
            # 훈련 히스토리에 대한 요약 텍스트 로깅
            summary_text = (
                f"오토인코더 사전 훈련 요약:\n"
                f"훈련 에포크: {len(history['vehicle_loss'])}\n"
                f"최종 차량 손실: {history['vehicle_loss'][-1]:.6f}\n"
                f"최종 자차 손실: {history['ego_loss'][-1]:.6f}\n"
                f"최종 공간 손실: {history['space_loss'][-1]:.6f}\n"
                f"최저 차량 손실: {min(history['vehicle_loss']):.6f} (에포크 {np.argmin(history['vehicle_loss'])})\n"
                f"최저 자차 손실: {min(history['ego_loss']):.6f} (에포크 {np.argmin(history['ego_loss'])})\n"
                f"최저 공간 손실: {min(history['space_loss']):.6f} (에포크 {np.argmin(history['space_loss'])})"
            )
            tf.summary.text('autoencoder/training_summary', summary_text, step=0)
            
            # 손실 감소 추세 시각화를 위한 데이터 로깅
            vehicle_loss_trend = np.array(history['vehicle_loss'])
            ego_loss_trend = np.array(history['ego_loss'])
            space_loss_trend = np.array(history['space_loss'])
            
            # 각 에포크 손실 대비 초기 손실의 비율 계산 (학습 진행도 확인)
            if len(vehicle_loss_trend) > 1 and vehicle_loss_trend[0] > 0:
                tf.summary.scalar('autoencoder/vehicle_loss_reduction', 
                                vehicle_loss_trend[-1] / vehicle_loss_trend[0], 
                                step=0)
            if len(ego_loss_trend) > 1 and ego_loss_trend[0] > 0:
                tf.summary.scalar('autoencoder/ego_loss_reduction', 
                                ego_loss_trend[-1] / ego_loss_trend[0], 
                                step=0)
            if len(space_loss_trend) > 1 and space_loss_trend[0] > 0:
                tf.summary.scalar('autoencoder/space_loss_reduction', 
                                space_loss_trend[-1] / space_loss_trend[0], 
                                step=0)
            
            # 훈련 완료 후 flush 호출로 모든 로그 저장 확인
            self.summary_writer.flush()
        
        # 오토인코더 가중치 고정
        self.freeze_autoencoder_weights()
        
        # 사전 훈련 완료 플래그 설정
        self.is_autoencoder_pretrained = True
        
        # TensorBoard 로그 위치 안내 메시지
        print(f"TensorBoard에서 훈련 결과를 확인하세요: tensorboard --logdir={os.path.dirname(self.log_dir)}")
        
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

    def train_dqn_step(self, states, actions, rewards, next_states, dones):
        """DQN 부분만 훈련하는 스텝"""
        # 모든 DQN 관련 변수 추출
        dqn_vars = []
        for var in self.model.trainable_variables:
            if not any(name in var.name for name in ['vehicle_autoencoder', 'ego_autoencoder', 'space_autoencoder']):
                dqn_vars.append(var)
        
        with tf.GradientTape() as tape:
            # 현재 상태 Q-값
            predicts = self.model(states)
            one_hot_action = tf.one_hot(actions, self.action_size)
            predicts = tf.reduce_sum(one_hot_action * predicts, axis=1)

            # 다음 상태 Q-값 (타겟 네트워크에서)
            target_predicts = self.target_model(next_states)
            target_predicts = tf.stop_gradient(target_predicts)

            # 벨만 방정식으로 타겟 Q-값 계산
            max_q = np.amax(target_predicts, axis=-1)
            targets = rewards + (1 - dones) * self.discount_factor * max_q
            
            # DQN 손실 계산
            loss = tf.reduce_mean(tf.square(targets - predicts))

        # 그래디언트 계산 및 모델 업데이트
        grads = tape.gradient(loss, dqn_vars)
        self.optimizer.apply_gradients(zip(grads, dqn_vars))
        
        # 로그 기록
        with self.summary_writer.as_default():
            tf.summary.scalar('training/dqn_loss', loss, step=self.train_step_counter)
            self.train_step_counter += 1
            
        return loss.numpy()

    def generate_training_data(self, num_samples=1000):
        """오토인코더 사전 훈련용 데이터 생성"""
        print("훈련 데이터 생성 중...")
        
        # 데이터 저장용 배열 초기화
        train_data = []
        
        # 환경에서 데이터 수집
        state = self.env.reset()
        state = np.reshape(state, [1, self.state_size])
        
        # 시퀀스 메모리 초기화
        state_memory = np.ones((self.sequence_length, self.state_size)) * 5
        state_memory[self.sequence_length-1] = state
        
        sample_count = 0
        max_steps = num_samples * 10  # 충분한 스텝 수 확보
        step_count = 0
        last_action = 5
        while sample_count < num_samples and step_count < max_steps:
            # 랜덤 행동 선택
            if round(self.env.get_simulation_time(), 2).is_integer():
                action = np.random.randint(self.action_size)
                last_action = action
            else:
                action = last_action
            
            # 환경에서 스텝 진행
            next_state, reward, done = self.env.step(action)
            next_state = np.reshape(next_state, [1, self.state_size])
            
            # 상태 메모리 업데이트
            state_memory = np.roll(state_memory, -1, axis=0)
            state_memory[self.sequence_length-1] = next_state
            
            # 유효한 상태 시퀀스만 저장 (초기화 값이 아닌 경우)
            if not np.all(state_memory[0] == 5):
                train_data.append(state_memory.copy())
                sample_count += 1
                
                if sample_count % 100 == 0:
                    print(f"데이터 생성 진행: {sample_count}/{num_samples}")
            
            # 에피소드 종료 시 초기화
            if done:
                self.env.end()
                state = self.env.reset()
                state = np.reshape(state, [1, self.state_size])
                state_memory = np.ones((self.sequence_length, self.state_size)) * 5
                state_memory[self.sequence_length-1] = state
            else:
                state = next_state
            
            step_count += 1
        self.env.end()
        # 결과 형식 맞추기
        train_data = np.array(train_data)
        print(f"훈련 데이터 생성 완료: {train_data.shape}")
        
        return train_data

    def _check_autoencoder_trainable_status(self):
        """오토인코더 레이어의 훈련 가능 상태를 확인하는 메서드"""
        # 차량 오토인코더 확인
        print("차량 오토인코더 레이어 훈련 상태:")
        for i, layer in enumerate(self.model.vehicle_autoencoder.layers):
            print(f"  레이어 {i} ({layer.__class__.__name__}): trainable = {layer.trainable}")
        
        # 자차 오토인코더 확인
        print("\n자차 오토인코더 레이어 훈련 상태:")
        for i, layer in enumerate(self.model.ego_autoencoder.layers):
            print(f"  레이어 {i} ({layer.__class__.__name__}): trainable = {layer.trainable}")
        
        # 공간 오토인코더 확인
        print("\n공간 오토인코더 레이어 훈련 상태:")
        for i, layer in enumerate(self.model.space_autoencoder.layers):
            print(f"  레이어 {i} ({layer.__class__.__name__}): trainable = {layer.trainable}")
        
        # 총 훈련 가능한 변수와 고정된 변수 수 계산
        total_params = 0
        trainable_params = 0
        
        for var in self.model.trainable_variables:
            trainable_params += np.prod(var.shape)
        
        for var in self.model.variables:
            total_params += np.prod(var.shape)
        
        print(f"\n총 모델 파라미터: {total_params:,}")
        print(f"훈련 가능한 파라미터: {trainable_params:,}")
        print(f"고정된 파라미터: {total_params - trainable_params:,}")
        print(f"훈련 가능 비율: {trainable_params / total_params:.2%}")

    def train(self):
        """메인 훈련 루프"""
        # 1. 초기 단계: 오토인코더 사전 훈련
        if self.using_external_agent:
            # 외부 에이전트의 오토인코더 사전 훈련 상태 확인
            if hasattr(self.agent, 'is_autoencoder_pretrained') and not self.agent.is_autoencoder_pretrained:
                print("===== 초기 단계: 외부 에이전트 오토인코더 사전 훈련 =====")
                # 훈련 데이터 생성
                train_data = self.generate_training_data(num_samples=5000)
                # 에이전트의 오토인코더 사전 훈련 메서드 호출
                self.agent.pretrain_autoencoder(train_data, epochs=30)
                self.is_autoencoder_pretrained = True  # 트레이너 상태 동기화
        elif not self.is_autoencoder_pretrained:
            print("===== 초기 단계: 내부 오토인코더 사전 훈련 =====")
            # 훈련 데이터 생성
            train_data = self.generate_training_data(num_samples=5000)
            # 오토인코더 사전 훈련
            self.pretrain_autoencoders(train_data, epochs=30)
        
         # 중간 단계 시작 전 타겟 모델과 원본 모델 동기화 확인
        print("===== 중간 단계: DQN 부분 훈련 =====")
        self.env.collision_num = 0
        for ep in range(self.config.MAX_EPISODES):
            # print(f"\n===== 에피소드 {ep} 후 오토인코더 trainable 상태 확인 =====")
            # self._check_autoencoder_trainable_status()
            # 에피소드 변수 초기화
            state = self.env.reset()
            state = np.reshape(state, [1, self.state_size])
            episode_reward = 0
            reward_memory = 0
            done = False
            
            # 시퀀스 상태 메모리 초기화
            state_memory = np.ones((self.sequence_length - 1, self.state_size)) * 5
            next_state_memory = np.ones((self.sequence_length - 2, self.state_size)) * 5
            next_state_memory = np.append(next_state_memory, state, axis=0)
            
            last_action = None
            
            # 에피소드 루프
            while not done:
                # 의사결정 시점 확인
                if round(self.env.get_simulation_time(), 2).is_integer():
                    # 행동 선택
                    if self.using_external_agent:
                        action = self.agent.get_action(state_memory)
                    else:
                        action = self.get_action(state_memory)

                    print('action:::::::: ', action)
                    # 환경에서 행동 실행
                    next_state, reward, done = self.env.step(action)
                    reward_memory += reward
                    
                    # 다음 상태 처리
                    next_state = np.reshape(next_state, [1, self.state_size])
                    
                    # 다음 상태 메모리 업데이트
                    if len(next_state_memory) > self.sequence_length - 1:
                        next_state_memory = np.delete(next_state_memory, 0, axis=0)
                    next_state_memory = np.append(next_state_memory, next_state, axis=0)
                    
                    episode_reward += reward_memory
                    
                    # 경험 저장 (유효한 시퀀스만)
                    if not np.all(state_memory[0] == 0) and not np.all(state_memory[0] == 5):
                        if self.using_external_agent:
                            self.agent.append_sample(state_memory, action, reward_memory, next_state_memory, done)
                        else:
                            self.append_sample(state_memory, action, reward_memory, next_state_memory, done)
                            self.consolidate_memory()
                    
                    # 모델 학습
                    if self.using_external_agent:
                        memory_size = (
                            self.agent.memory.tree.n_entries 
                            if hasattr(self.agent.memory, 'tree') 
                            else len(self.agent.memory.buffer) 
                            if hasattr(self.agent.memory, 'buffer') 
                            else 0
                        )
                        if memory_size >= self.config.TRAIN_START:
                            loss = self.agent.train_model()
                            with self.summary_writer.as_default():
                                tf.summary.scalar('training/loss', loss, step=self.train_step_counter)
                                self.train_step_counter += 1
                    else:
                        if len(self.memory) >= self.config.TRAIN_START:
                            # 배치 샘플링 및 학습
                            mini_batch = random.sample(self.memory, self.batch_size)
                            
                            states = np.array([sample[0] for sample in mini_batch])
                            actions = np.array([sample[1] for sample in mini_batch])
                            rewards = np.array([sample[2] for sample in mini_batch])
                            next_states = np.array([sample[3] for sample in mini_batch])
                            dones = np.array([sample[4] for sample in mini_batch])
                            
                            # DQN만 학습 (오토인코더는 고정)
                            loss = self.train_dqn_step(states, actions, rewards, next_states, dones)

                    # 상태 업데이트
                    state = next_state
                    
                    # 상태 메모리 업데이트
                    if len(state_memory) > self.sequence_length - 1:
                        state_memory = np.delete(state_memory, 0, axis=0)
                    state_memory = np.append(state_memory, state, axis=0)
                    
                    last_action = action
                    reward_memory = 0
                else:
                    # 의사결정 시점 사이에는 마지막 행동 지속
                    next_state, reward, done = self.env.step(last_action)
                    if done:
                        next_state = np.reshape(next_state, [1, self.state_size])
                        
                        # 메모리 업데이트
                        if len(next_state_memory) > self.sequence_length - 1:
                            next_state_memory = np.delete(next_state_memory, 0, axis=0)
                        next_state_memory = np.append(next_state_memory, next_state, axis=0)
                        
                        state = next_state
                        
                        if len(state_memory) > self.sequence_length - 1:
                            state_memory = np.delete(state_memory, 0, axis=0)
                        state_memory = np.append(state_memory, state, axis=0)
                    
                    reward_memory += reward
                
                # 에피소드 종료 처리
                if done:
                    if (reward_memory != 0): # 중복 처리 방지
                        # 유효한 시퀀스인지 확인
                        if not np.all(state_memory[0] == 0) and not np.all(state_memory[0] == 5):
                            print("마지막 reward 기록 중... reward:", reward_memory)
                            if self.using_external_agent:
                                self.agent.append_sample(state_memory, last_action, reward_memory, next_state_memory, done)
                            else:
                                self.append_sample(state_memory, last_action, reward_memory, next_state_memory, done)
                                self.consolidate_memory()
                        episode_reward += reward_memory
                    
                    # 주기적으로 모델 가중치 히스토그램 로깅
                    if ep % 50 == 0:
                        self.log_histogram_weights(ep)
                        
                    # 에피소드 카운터 증가
                    self.episode_counter += 1
                        
                    # 외부 에이전트 처리
                    if self.using_external_agent and hasattr(self.agent, 'end_episode'):
                        current_epsilon = self.agent.end_episode(episode_reward)
                        self.epsilon = current_epsilon
                    else:
                        # 기존 방식 유지 (10 에피소드마다 입실론 감소)
                        if self.epsilon > self.epsilon_min and self.episode_counter % self.epsilon_update_frequency == 0:
                            old_epsilon = self.epsilon
                            self.epsilon *= self.epsilon_decay
                            print(f"Epsilon decreased from {old_epsilon:.4f} to {self.epsilon:.4f}")
                    
                    # 메트릭 계산
                    self.avg_reward = 0.9 * self.avg_reward + 0.1 * episode_reward if self.avg_reward != 0 else episode_reward
                    
                    # 메모리 크기 확인
                    memory_size = (
                        self.agent.memory.tree.n_entries 
                        if self.using_external_agent and hasattr(self.agent.memory, 'tree')
                        else len(self.agent.memory.buffer) 
                        if self.using_external_agent and hasattr(self.agent.memory, 'buffer')
                        else len(self.memory) if not self.using_external_agent
                        else 0
                    )
                    
                    # 타겟 네트워크 주기적 업데이트
                    if memory_size >= self.config.TRAIN_START and ep % 1 == 0:
                        self._debug_model_layers()  # 디버그 정보 출력
                        self.update_target_network(initial_update = False)
                        
                    # 후방 차량 특성 출력
                    if self.env.rear_collision_mode == 1:
                        print("Rear vehicle: Unattentive Vehicle(Red)")
                    else:
                        print("Rear vehicle: Normal vehicle(Blue)")
                        
                    # 에피소드 메트릭 출력
                    print(f"Episode: {ep:3d} | "
                            f"Episode reward: {episode_reward:3.0f} | "
                            f"Average reward: {self.avg_reward:3.2f} | "
                            f"Memory length: {memory_size:4d} | "
                            f"Epsilon: {self.epsilon:.4f}")
                    
                    if hasattr(self.env, 'collision_num'):
                        print(f"Episode: {ep} | Collision: {self.env.collision_num} | "
                                f"Space sizes: 1st: {self.env.space1_count}, 2nd: {self.env.space2_count}, "
                                f"3rd: {self.env.space3_count}, 4th: {self.env.space4_count}")
                        
                        print(f"Episode: {ep} | Collision: {self.env.collision_num} | "
                                f"Space locations: R1: {self.env.R1_count}, R2: {self.env.R2_count}, "
                                f"L1: {self.env.L1_count}, L2: {self.env.L2_count}")
                    
                    # TensorBoard에 메트릭 로깅
                    with self.summary_writer.as_default():
                        # 보상
                        tf.summary.scalar('evaluation/reward', episode_reward, step=ep)
                        tf.summary.scalar('evaluation/avg_reward', self.avg_reward, step=ep)
                        
                        # 메모리 사용량
                        tf.summary.scalar('memory/total_size', memory_size, step=ep)
                        
                        if not self.using_external_agent:
                            tf.summary.scalar('memory/action0_size', len(self.memory0), step=ep) 
                            tf.summary.scalar('memory/action1_size', len(self.memory1), step=ep)
                            tf.summary.scalar('memory/action2_size', len(self.memory2), step=ep)
                            tf.summary.scalar('memory/action3_size', len(self.memory3), step=ep)
                            tf.summary.scalar('memory/action4_size', len(self.memory4), step=ep)
                            tf.summary.scalar('memory/action5_size', len(self.memory5), step=ep)
                        
                        # 학습 파라미터
                        tf.summary.scalar('training/epsilon', self.epsilon, step=ep)
                        
                        # 환경 메트릭 (존재하는 경우)
                        if hasattr(self.env, 'collision_num'):
                            tf.summary.scalar('evaluation/collisions', self.env.collision_num, step=ep)
                            
                            # 공간 크기 메트릭
                            tf.summary.scalar('environment/space1_count', self.env.space1_count, step=ep)
                            tf.summary.scalar('environment/space2_count', self.env.space2_count, step=ep)
                            tf.summary.scalar('environment/space3_count', self.env.space3_count, step=ep)
                            tf.summary.scalar('environment/space4_count', self.env.space4_count, step=ep)
                            
                            # 공간 위치 메트릭
                            tf.summary.scalar('environment/R1_count', self.env.R1_count, step=ep)
                            tf.summary.scalar('environment/R2_count', self.env.R2_count, step=ep)
                            tf.summary.scalar('environment/L1_count', self.env.L1_count, step=ep)
                            tf.summary.scalar('environment/L2_count', self.env.L2_count, step=ep)
                    
                    # 성능 향상 시 모델 저장
                    if self.avg_reward > self.best_reward:
                        self.best_reward = self.avg_reward
                        self._save_weights(best=True)
                    
                    # 주기적 모델 저장
                    if ep % self.config.SAVE_INTERVAL == 0:
                        self._save_weights(best=False)
                        
                    # 혼동 행렬 저장 (존재하는 경우)
                    if hasattr(self.env, 'confusion_matrix_data_collection'):
                        self.env.confusion_matrix_data_collection(ep)
                    
                    # 환경 종료 (필요시)
                    if hasattr(self.env, 'end'):
                        self.env.end()
                    
                    # TensorBoard 요약 저장
                    if ep % 10 == 0 and not ep == 0:
                        self.summary_writer.flush()
                        
                    # 종료 조건 확인
                    if self.avg_reward > self.config.REWARD_THRESHOLD:
                        print("\nProblem solved!")
                        self._save_weights(best=True)
                        break

    def _save_weights(self, best=False):
        """모델 가중치 저장"""
        prefix = "best_" if best else ""
        agent_type = self.agent_name if self.using_external_agent else "lstm_autoencoder_dqn"
        
        os.makedirs(self.config.WEIGHTS_PATH, exist_ok=True)
        
        print(f"Current working directory: {os.getcwd()}")
        print(f"Saving weights to: {os.path.abspath(self.config.WEIGHTS_PATH)}")
        
        filename = f"{prefix}{agent_type}_model.h5"
        full_path = os.path.join(self.config.WEIGHTS_PATH, filename)
        self.model.save_weights(full_path)
    
    def _save_autoencoder_weights(self, epoch):
        os.makedirs(self.config.WEIGHTS_PATH, exist_ok=True)
        save_dir = self.config.WEIGHTS_PATH
        """오토인코더 가중치만 별도 저장"""
        # 차량 오토인코더 저장
        self.model.vehicle_autoencoder.save_weights(f"{save_dir}/vehicle_autoencoder_ep{epoch+1}.h5")
        # 자차 오토인코더 저장
        self.model.ego_autoencoder.save_weights(f"{save_dir}/ego_autoencoder_ep{epoch+1}.h5")
        # 공간 오토인코더 저장
        self.model.space_autoencoder.save_weights(f"{save_dir}/space_autoencoder_ep{epoch+1}.h5")
    
    # def load_autoencoder_weights(self, vehicle_weights_path, ego_weights_path, space_weights_path):
    #     """사전 훈련된 오토인코더 가중치 로드"""
    #     print("사전 훈련된 오토인코더 가중치 로드 중...")
    #     self.model.vehicle_autoencoder.load_weights(vehicle_weights_path)
    #     self.model.ego_autoencoder.load_weights(ego_weights_path)
    #     self.model.space_autoencoder.load_weights(space_weights_path)
        
    #     # 타겟 모델에도 가중치 복사
    #     self.update_target_network()
        
    #     # 오토인코더 가중치 고정
    #     self.freeze_autoencoder_weights()
        
    #     # 사전 훈련 완료 플래그 설정
    #     self.is_autoencoder_pretrained = True
    #     print("오토인코더 가중치 로드 완료")
    
    def log_histogram_weights(self, episode):
        """TensorBoard에 모델 가중치 히스토그램 로깅"""
        if episode % 50 == 0:  # 공간 절약을 위해 주기적으로만 히스토그램 로깅
            with self.summary_writer.as_default():
                # 모델 가중치 히스토그램 로깅
                for i, layer in enumerate(self.model.layers):
                    for weight in layer.weights:
                        tf.summary.histogram(
                            f"weights/{layer.name}/{weight.name}", 
                            weight, 
                            step=episode
                        )
                
                # 오토인코더 가중치도 로깅
                for encoder_name, encoder in [
                    ('vehicle', self.model.vehicle_autoencoder),
                    ('ego', self.model.ego_autoencoder),
                    ('space', self.model.space_autoencoder)
                ]:
                    for layer in encoder.layers:
                        for weight in layer.weights:
                            tf.summary.histogram(
                                f"autoencoder/{encoder_name}/{layer.name}/{weight.name}",
                                weight,
                                step=episode
                            )
                
                self.summary_writer.flush()
    
    def _setup_tensorboard(self):
        """포괄적인 로깅을 위한 TensorBoard 설정"""
        # 타임스탬프가 있는 고유 디렉토리 이름 생성
        current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
        
        # 로그 디렉토리 구조 생성
        log_base_dir = os.path.join(self.config.LOG_PATH, 'DQN')
        self.log_dir = os.path.join(log_base_dir, current_time)
        
        # 다양한 메트릭 카테고리를 위한 하위 디렉토리 생성
        os.makedirs(os.path.join(self.log_dir, 'training'), exist_ok=True)
        os.makedirs(os.path.join(self.log_dir, 'autoencoder'), exist_ok=True)
        os.makedirs(os.path.join(self.log_dir, 'episode'), exist_ok=True)
        os.makedirs(os.path.join(self.log_dir, 'memory'), exist_ok=True)
        os.makedirs(os.path.join(self.log_dir, 'model'), exist_ok=True)
        os.makedirs(os.path.join(self.log_dir, 'environment'), exist_ok=True)
        os.makedirs(os.path.join(self.log_dir, 'evaluation'), exist_ok=True)
        
        # 요약 작성기 생성
        self.summary_writer = tf.summary.create_file_writer(self.log_dir)
        print(f"TensorBoard 로그가 저장될 위치: {self.log_dir}")
        print(f"로그를 보려면 다음을 실행하세요: tensorboard --logdir={log_base_dir}")
        
        # 하이퍼파라미터 로깅
        with self.summary_writer.as_default():
            # 텍스트로 하이퍼파라미터 로깅
            discount_factor = self.discount_factor if hasattr(self, 'discount_factor') else self.config.DISCOUNT_FACTOR
            epsilon_decay = self.epsilon_decay if hasattr(self, 'epsilon_decay') else self.config.EPSILON_DECAY
            epsilon_min = self.epsilon_min if hasattr(self, 'epsilon_min') else self.config.EPSILON_MIN
            
            hyperparams = (
                f"할인 인자: {discount_factor}\n"
                f"학습률: {self.config.LEARNING_RATE}\n"
                f"입실론 시작: {self.epsilon}\n"
                f"입실론 감소율: {epsilon_decay}\n"
                f"입실론 최소값: {epsilon_min}\n"
                f"배치 크기: {self.batch_size}\n"
                f"시퀀스 길이: {self.sequence_length}\n"
                f"상태 크기: {self.state_size}\n"
                f"액션 크기: {self.action_size}\n"
                f"오토인코더 사용: True\n"
                f"외부 에이전트 사용: {self.using_external_agent}\n"
            )
            tf.summary.text('hyperparameters', hyperparams, step=0)
            
            # 텍스트로 네트워크 아키텍처 로깅 - 모델이 빌드된 경우에만 수행
            try:
                model_summary = []
                self.model.summary(print_fn=lambda x: model_summary.append(x))
                tf.summary.text('model_architecture', '\n'.join(model_summary), step=0)
            except ValueError as e:
                print(f"모델 아키텍처 로깅 건너뜀: {e}")
                tf.summary.text('model_architecture', "모델이 아직 빌드되지 않았습니다.", step=0)
            
            self.summary_writer.flush()
        
        print(f"TensorBoard 로그가 저장될 위치: {self.log_dir}")
        print(f"로그를 보려면 다음을 실행하세요: tensorboard --logdir={log_base_dir}")