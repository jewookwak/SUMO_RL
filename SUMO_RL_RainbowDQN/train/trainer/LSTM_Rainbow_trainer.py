# train/LSTM_Rainbow_trainer.py
import os
import numpy as np
import tensorflow as tf
from tensorflow.keras.optimizers import Adam
from datetime import datetime
import random
from train.agent.lstm_dqn_rainbow_agent import LSTMDQNRainbowAgent
import traci

class LSTMRainbowTrainer:
    def __init__(self, env, config, agent=None, agent_name=None):
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
        self.sequence_length = config.SEQUENCE_LENGTH
        self.batch_size = config.BATCH_SIZE
        
        # 초기화를 위한 변수들
        self.best_reward = float('-inf')
        self.avg_reward = 0
        self.episode_counter = 0
        
        # Rainbow DQN 에이전트 초기화 - 외부에서 제공된 에이전트가 있으면 사용
        if agent is not None:
            self.agent = agent
        else:
            # 기존처럼 에이전트를 내부에서 생성
            self.agent = LSTMDQNRainbowAgent(self.state_size, self.action_size, config)
        self.agent_name = agent_name    
        self.model = self.agent.model
        self.target_model = self.agent.target_model
        self.optimizer = self.agent.optimizer
        self.epsilon = self.agent.epsilon
        
        # TensorBoard setup
        self._setup_tensorboard()

    def update_target_network(self):
        """타깃 네트워크 업데이트"""
        self.agent.update_target_model()
            
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
                f"N-step: {self.config.N_STEP}\n"
                f"IQN Embedding Dim: {self.config.IQN_EMBEDDING_DIM}\n"
                f"Use Noisy Network: {self.config.USE_NOISY_NETWORK if hasattr(self.config, 'USE_NOISY_NETWORK') else True}\n"
            )
            tf.summary.text('hyperparameters', hyperparams, step=0)

    def train(self):
        """주요 학습 루프"""
        for ep in range(self.config.MAX_EPISODES):
            # 에피소드 변수 초기화
            state = self.env.reset()
            state = np.reshape(state, [1, self.state_size])
            episode_reward = 0
            reward_memory = 0
            done = False
            
            # 시퀀스를 위한 상태 메모리 초기화
            state_memory = np.ones((self.sequence_length - 1, self.state_size)) * 5
            next_state_memory = np.ones((self.sequence_length - 2, self.state_size)) * 5
            next_state_memory = np.append(next_state_memory, state, axis=0)
            
            last_action = None
            
            # 에피소드 루프
            while not done:
                # 결정을 내릴 시간인지 확인
                if round(self.env.get_simulation_time(), 2).is_integer():
                    # Rainbow DQN 에이전트로 행동 선택
                    action = self.agent.get_action(state_memory)
                    print('action:::::::: ', action)
                    # 환경에서 행동 수행
                    next_state, reward, done = self.env.step(action)
                    reward_memory += reward
                    
                    # 다음 상태 처리
                    next_state = np.reshape(next_state, [1, self.state_size])
                    
                    # 다음 상태 메모리 업데이트
                    if len(next_state_memory) > self.sequence_length - 1:
                        next_state_memory = np.delete(next_state_memory, 0, axis=0)
                    next_state_memory = np.append(next_state_memory, next_state, axis=0)
                    
                    episode_reward += reward_memory
                    # 경험 저장 (시퀀스가 비어있지 않은 경우에만)
                    if not np.all(state_memory[0] == 0) and not np.all(state_memory[0] == 5):
                        self.agent.append_sample(state_memory, action, reward_memory, next_state_memory, done)
                    
                    # 모델 학습
                    memory_size = self.agent.memory.tree.n_entries if hasattr(self.agent.memory, 'tree') else 0
                    if memory_size >= self.config.TRAIN_START:
                        loss = self.agent.train_model()
                        with self.summary_writer.as_default():
                            tf.summary.scalar('training/loss', loss, step=self.train_step_counter)
                            self.train_step_counter += 1

                    # 상태 업데이트
                    state = next_state
                    
                    # 상태 메모리 업데이트
                    if len(state_memory) > self.sequence_length - 1:
                        state_memory = np.delete(state_memory, 0, axis=0)
                    state_memory = np.append(state_memory, state, axis=0)
                    
                    last_action = action
                    reward_memory = 0
                else:
                    # 결정 지점 사이에서는 마지막 행동 계속 수행
                    next_state, reward, done = self.env.step(last_action)
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
                    #Terminal state and reward
                    if (reward_memory !=0): #rewardmemory가 0인 것은 앞에서 terminal reward가 메모리에 이미 담긴것으로 중복 처리 할 필요 없음
                        if not np.all(state_memory[0] == 0) and not np.all(state_memory[0] == 5):
                            self.agent.append_sample(state_memory, action, reward_memory, next_state_memory, done)
                            print("마지막 reward 기록 중... reward:", reward_memory)

                        episode_reward += reward_memory
                    # 주기적인 모델 가중치 히스토그램 로깅
                    if ep % 50 == 0:
                        self.log_histogram_weights(ep)
                        
                    # 에피소드 카운터 증가
                    self.episode_counter += 1
                    
                    # 에이전트의 end_episode 메서드 호출
                    self.epsilon = self.agent.end_episode(episode_reward)
                    
                    # 메트릭 계산
                    self.avg_reward = 0.9 * self.avg_reward + 0.1 * episode_reward if self.avg_reward != 0 else episode_reward
                    
                    # 메모리 크기 파악
                    memory_size = self.agent.memory.tree.n_entries if hasattr(self.agent.memory, 'tree') else 0
                    
                    # 타깃 네트워크 주기적 업데이트
                    if memory_size >= self.config.TRAIN_START and ep % 1 == 0:
                        self.update_target_network()
                    
                    # Rear vehicle character
                    if self.env.rear_collision_mode == 1:
                        print("Rear vehicle: Unattentive Vehicle(Red)")
                    else:
                        print("Rear vehicle: Normal vehicle(Blue)")

                    # 에피소드 메트릭 로깅
                    print(f"Episode: {ep:3d} | "
                          f"Episode reward: {episode_reward:3.0f} | "
                          f"Average reward: {self.avg_reward:3.2f} | "
                          f"Memory length: {memory_size:4d} | "
                          f"Epsilon: {self.epsilon:.4f}")
                    
                    # 충돌 수와 공간 크기 출력 (가능한 경우)
                    if hasattr(self.env, 'collision_num'):
                        print(f"Episode: {ep} | Collision: {self.env.collision_num} | "
                              f"Space sizes: 1st: {self.env.space1_count}, 2nd: {self.env.space2_count}, "
                              f"3rd: {self.env.space3_count}, 4th: {self.env.space4_count}")
                        
                        print(f"Episode: {ep} | Collision: {self.env.collision_num} | "
                              f"Space locations: R1: {self.env.R1_count}, R2: {self.env.R2_count}, "
                              f"L1: {self.env.L1_count}, L2: {self.env.L2_count}")
                    
                    # TensorBoard에 종합적인 에피소드 메트릭 로깅
                    with self.summary_writer.as_default():
                        # 보상
                        tf.summary.scalar('evaluation/reward', episode_reward, step=ep)
                        tf.summary.scalar('evaluation/avg_reward', self.avg_reward, step=ep)
                        
                        # 메모리 사용량
                        tf.summary.scalar('memory/total_size', memory_size, step=ep)
                        
                        # 학습 파라미터
                        tf.summary.scalar('training/epsilon', self.epsilon, step=ep)
                        
                        # 환경 메트릭 (가능한 경우)
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
                    
                    # 모델이 향상된 경우 저장
                    if self.avg_reward > self.best_reward:
                        self.best_reward = self.avg_reward
                        self._save_weights(best=True)
                    
                    # 주기적인 모델 저장
                    if ep % self.config.SAVE_INTERVAL == 0:
                        self._save_weights(best=False)
                        
                    # 혼동 행렬 저장 (가능한 경우)
                    if hasattr(self.env, 'confusion_matrix_data_collection'):
                        self.env.confusion_matrix_data_collection(ep)
                    
                    # 필요한 경우 환경 종료
                    if hasattr(self.env, 'end'):
                        self.env.end()
                    
                    # 중요한 에피소드에 대한 TensorBoard 요약 자주 저장
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
        agent_type = self.agent_name if self.agent else "lstm_dqn_rainbow"
        os.makedirs(self.config.WEIGHTS_PATH, exist_ok=True)
        
        print(f"Current working directory: {os.getcwd()}")
        print(f"Saving weights to: {os.path.abspath(self.config.WEIGHTS_PATH)}")
        
        filename = f"{prefix}{agent_type}_model.h5"
        full_path = os.path.join(self.config.WEIGHTS_PATH, filename)
        self.model.save_weights(full_path)
        
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
        self._log_hyperparameters()
            
        # 텍스트로 네트워크 아키텍처 로깅 - 모델이 빌드된 경우에만 수행
        with self.summary_writer.as_default():
            try:
                model_summary = []
                self.model.summary(print_fn=lambda x: model_summary.append(x))
                tf.summary.text('model_architecture', '\n'.join(model_summary), step=0)
            except ValueError as e:
                print(f"모델 아키텍처 로깅 건너뜀: {e}")
                tf.summary.text('model_architecture', "모델이 아직 빌드되지 않았습니다.", step=0)
            
            self.summary_writer.flush()
            
    def get_simulation_time(self):
        """시뮬레이션 시간을 가져오는 헬퍼 메서드"""
        if hasattr(self.env, 'get_simulation_time'):
            return self.env.get_simulation_time()
        else:
            import traci
            return np.round(traci.simulation.getTime(), 2)