# train/trainer.py
import os
import numpy as np
import tensorflow as tf
from tensorflow.keras.optimizers import Adam
from datetime import datetime

from train.network.networks import DQN
from train.replay_buffer import ReplayBuffer

class DQNTrainer:
    def __init__(self, env, config, agent=None,agent_name=None):
        self.env = env
        self.config = config
        self.train_step_counter = 0
        
        # Create save directories
        os.makedirs(self.config.WEIGHTS_PATH, exist_ok=True)
        os.makedirs(self.config.GRAPH_PATH, exist_ok=True)
        os.makedirs(self.config.LOG_PATH, exist_ok=True)
        
        # 에이전트 초기화 방식 결정
        if agent is not None:
            self.agent = agent
            self.model = agent.model
            self.target_model = agent.target_model
            self.optimizer = agent.optimizer if hasattr(agent, 'optimizer') else Adam(learning_rate=self.config.LEARNING_RATE)
            self.epsilon = agent.epsilon if hasattr(agent, 'epsilon') else self.config.EPSILON
            self.epsilon_decay = agent.epsilon_decay if hasattr(agent, 'epsilon_decay') else self.config.EPSILON_DECAY
            self.epsilon_min = agent.epsilon_min if hasattr(agent, 'epsilon_min') else self.config.EPSILON_MIN
            self.discount_factor = agent.discount_factor if hasattr(agent, 'discount_factor') else self.config.DISCOUNT_FACTOR
            self.using_external_agent = True
            self.agent_name = agent_name
        else:
            # 기존 DQN 모델 초기화
            self.model = DQN(len(env.observation_space), len(env.action_space))
            self.target_model = DQN(len(env.observation_space), len(env.action_space))
            self.optimizer = Adam(learning_rate=self.config.LEARNING_RATE)
            
            # 초기화 변수들
            self.epsilon = self.config.EPSILON
            self.epsilon_decay = self.config.EPSILON_DECAY
            self.epsilon_min = self.config.EPSILON_MIN
            self.discount_factor = self.config.DISCOUNT_FACTOR
            self.buffer = ReplayBuffer(self.config.MEMORY_SIZE)
            self.using_external_agent = False
            
            # 에피소드 카운터와 입실론 감소 주기 설정
            self.episode_counter = 0
            self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소
        
        # 공통 초기화 변수들
        self.best_reward = float('-inf')
        
        # 메트릭스 초기화
        self.rewards = []
        self.episodes = []
        self.avg_reward = 0
        
        # TensorBoard setup
        self._setup_tensorboard()
        
        # 타깃 네트워크 초기화
        self.update_target_network()

    def update_target_network(self):
        """Update target network with weights from main network"""
        if self.using_external_agent:
            self.agent.update_target_model()
        else:
            self.target_model.set_weights(self.model.get_weights())

    def get_action(self, state):
        """Choose action using epsilon-greedy policy"""
        if self.using_external_agent:
            return self.agent.get_action(state)
        else:
            if np.random.rand() <= self.epsilon:
                return np.random.randint(self.env.action_space.shape[0])
            else:
                q_value = self.model(state)
                return np.argmax(q_value[0])

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
        
        # 하이퍼파라미터 로깅
        self._log_hyperparameters()
        
        print(f"TensorBoard 로그가 저장될 위치: {self.log_dir}")
        print(f"로그를 보려면 다음을 실행하세요: tensorboard --logdir={log_base_dir}")

    def _log_hyperparameters(self):
        """로그에 하이퍼파라미터 기록"""
        with self.summary_writer.as_default():
            # 하이퍼파라미터 텍스트로 로깅
            hyperparams = (
                f"Learning Rate: {self.config.LEARNING_RATE}\n"
                f"Epsilon Start: {self.epsilon}\n"
                f"Epsilon Decay: {self.epsilon_decay}\n"
                f"Epsilon Min: {self.epsilon_min}\n"
                f"Discount Factor: {self.discount_factor}\n"
                f"Batch Size: {self.config.BATCH_SIZE}\n"
                f"Memory Size: {self.config.MEMORY_SIZE}\n"
                f"Using External Agent: {self.using_external_agent}\n"
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

    def get_memory_size(self):
        """Get the current memory size based on agent type"""
        if self.using_external_agent:
            if hasattr(self.agent, 'memory'):
                if hasattr(self.agent.memory, 'tree'):
                    return self.agent.memory.tree.n_entries
                elif hasattr(self.agent.memory, 'buffer'):
                    return len(self.agent.memory.buffer)
            elif hasattr(self.agent, 'buffer'):
                if hasattr(self.agent.buffer, 'tree'):
                    return self.agent.buffer.tree.n_entries
                elif hasattr(self.agent.buffer, 'buffer'):
                    return len(self.agent.buffer.buffer)
                else:
                    return self.agent.buffer.buffer_count()
            return 0
        else:
            return self.buffer.buffer_count()

    def train_step(self, states, actions, rewards, next_states, dones):
        """Perform a single training step on a batch of experiences"""
        model_params = self.model.trainable_variables
        with tf.GradientTape() as tape:
            # 현재 상태에 대한 모델의 큐함수
            predicts = self.model(states)
            one_hot_action = tf.one_hot(actions, self.env.action_space.shape[0])
            predicts = tf.reduce_sum(one_hot_action * predicts, axis=1)

            # 다음 상태에 대한 타깃 모델의 큐함수
            target_predicts = self.target_model(next_states)
            target_predicts = tf.stop_gradient(target_predicts)

            # 벨만 최적 방정식을 이용한 업데이트 타깃
            max_q = np.amax(target_predicts, axis=-1)
            targets = rewards + (1 - dones) * self.discount_factor * max_q
            loss = tf.reduce_mean(tf.square(targets - predicts))

        # 오류함수를 줄이는 방향으로 모델 업데이트
        grads = tape.gradient(loss, model_params)
        self.optimizer.apply_gradients(zip(grads, model_params))
        
        # 로그 기록 추가
        with self.summary_writer.as_default():
            tf.summary.scalar('training/loss', loss, step=self.train_step_counter)
            self.train_step_counter += 1
            
        return loss

    def train(self):
        for ep in range(self.config.MAX_EPISODES):
            state = self.env.reset()
            state = np.reshape(state, [1, len(self.env.observation_space)])
            episode_reward, done = 0, False
            while not done:
                action = self.get_action(state)
                print('action:::::::::::::', action)
                next_state, reward, done = self.env.step(action)
                next_state = np.reshape(next_state, [1, len(self.env.observation_space)])
                
                episode_reward += reward
                
                
                # 외부 에이전트 사용 여부에 따라 경험 저장 방식 결정
                if self.using_external_agent:
                    self.agent.append_sample(state[0], action, reward, next_state[0], done)
                else:
                    self.buffer.add_buffer(state, action, reward, next_state, done)
                
                # 학습 부분
                memory_size = self.get_memory_size()
                train_ready = memory_size >= self.config.TRAIN_START
                
                if train_ready:
                    if self.using_external_agent:
                        loss = self.agent.train_model()
                        # 에이전트가 loss 값을 반환하는 경우 로깅
                        if loss is not None:
                            # NumPy 배열을 스칼라 값으로 변환
                            if hasattr(loss, 'item'):  # PyTorch 텐서
                                loss = loss.item()
                            elif hasattr(loss, 'numpy'):  # TensorFlow 텐서
                                loss = float(loss.numpy())
                            elif isinstance(loss, np.ndarray):  # NumPy 배열
                                loss = float(loss)
                            else:  # 이미 스칼라 값
                                loss = loss
                            with self.summary_writer.as_default():
                                tf.summary.scalar('training/loss', loss, step=self.train_step_counter)
                                self.train_step_counter += 1
                    else:
                        states, actions, rewards, next_states, dones = self.buffer.sample_batch(
                            self.config.BATCH_SIZE)
                        self.train_step(states, actions, rewards, next_states, dones)
                
                state = next_state
                
                if done:
                    # 주기적으로 모델 가중치 히스토그램 로깅
                    if ep % 50 == 0:
                        self.log_histogram_weights(ep)
                        
                    # 에피소드 카운터 증가
                    if not self.using_external_agent:
                        self.episode_counter += 1
                    
                    # 타깃 네트워크 업데이트 및 메트릭스 계산
                    if memory_size >= self.config.TRAIN_START and ep % 1 == 0:
                        self.update_target_network()
                        
                    self.avg_reward = 0.9 * self.avg_reward + 0.1 * episode_reward if self.avg_reward != 0 else episode_reward
                    
                    # 메모리 크기 확인
                    memory_size = self.get_memory_size()
                    
                    # Rear vehicle character
                    if self.env.rear_collision_mode == 1:
                        print("Rear vehicle: Unattentive Vehicle(Red)")
                    else:
                        print("Rear vehicle: Normal vehicle(Blue)")

                    # 에피소드 정보 출력
                    print(f"episode: {ep:3d} | "
                          f"episode reward: {episode_reward:3.0f} | "
                          f"average reward: {self.avg_reward:3.2f} | "
                          f"memory length: {memory_size:4d} | "
                          f"epsilon: {self.epsilon:.4f}")
                    
                    # 에피소드 메트릭스 로깅
                    with self.summary_writer.as_default():
                        # Rewards
                        tf.summary.scalar('evaluation/reward', episode_reward, step=ep)
                        tf.summary.scalar('evaluation/avg_reward', self.avg_reward, step=ep)
                        
                        # Memory usage
                        tf.summary.scalar('memory/total_size', memory_size, step=ep)
                        
                        # Training parameters
                        tf.summary.scalar('training/epsilon', self.epsilon, step=ep)
                        
                        # Environment metrics if available
                        if hasattr(self.env, 'collision_num'):
                            tf.summary.scalar('evaluation/collisions', self.env.collision_num, step=ep)
                            
                            # Space size metrics if available
                            if hasattr(self.env, 'space1_count'):
                                tf.summary.scalar('environment/space1_count', self.env.space1_count, step=ep)
                                tf.summary.scalar('environment/space2_count', self.env.space2_count, step=ep)
                                tf.summary.scalar('environment/space3_count', self.env.space3_count, step=ep)
                                tf.summary.scalar('environment/space4_count', self.env.space4_count, step=ep)
                            
                            # Space location metrics if available
                            if hasattr(self.env, 'R1_count'):
                                tf.summary.scalar('environment/R1_count', self.env.R1_count, step=ep)
                                tf.summary.scalar('environment/R2_count', self.env.R2_count, step=ep)
                                tf.summary.scalar('environment/L1_count', self.env.L1_count, step=ep)
                                tf.summary.scalar('environment/L2_count', self.env.L2_count, step=ep)
                    
                    # 가중치 저장
                    if self.avg_reward > self.best_reward:
                        self.best_reward = self.avg_reward
                        self._save_weights(best=True)
                    
                    # 주기적인 모델 저장
                    if ep % self.config.SAVE_INTERVAL == 0:
                        self._save_weights(best=False)
                    
                    # 외부 에이전트를 사용하는 경우 end_episode 메서드 호출
                    if self.using_external_agent and hasattr(self.agent, 'end_episode'):
                        # end_episode 메서드 호출하고 반환값 받기 (입실론 값)
                        current_epsilon = self.agent.end_episode(episode_reward)
                        # 에이전트의 현재 입실론 값 사용
                        self.epsilon = current_epsilon
                    else:
                        # 기존 방식 유지 (10 에피소드마다 입실론 감소)
                        if self.epsilon > self.epsilon_min and self.episode_counter % self.epsilon_update_frequency == 0:
                            old_epsilon = self.epsilon
                            self.epsilon *= self.epsilon_decay
                            print(f"Epsilon decreased from {old_epsilon:.4f} to {self.epsilon:.4f}")
                    
                    # Save TensorBoard summaries more frequently for important episodes
                    if ep % 10 == 0 and not ep == 0:
                        self.summary_writer.flush()  # Make sure all summaries are written to disk
                    
                    # Check for termination condition
                    if self.avg_reward > self.config.REWARD_THRESHOLD:
                        print("\nProblem solved!")
                        self._save_weights(best=True)
                        break
            
            # 에피소드 종료 후 혼동 행렬 수집
            if hasattr(self.env, 'confusion_matrix_data_collection'):
                self.env.confusion_matrix_data_collection(ep)
            
            # 환경 종료
            if hasattr(self.env, 'end'):
                self.env.end()

    def _save_weights(self, best=False):
        """Save model weights"""
        prefix = "best_" if best else ""
        agent_type = self.agent_name if self.using_external_agent else "dqn"
        
        # 저장 경로 확인
        os.makedirs(self.config.WEIGHTS_PATH, exist_ok=True)
        
        # 디버깅 정보 출력
        print(f"Current working directory: {os.getcwd()}")
        print(f"Saving weights to: {os.path.abspath(self.config.WEIGHTS_PATH)}")
        
        # 모델 저장
        filename = f"{prefix}{agent_type}_model.h5"
        full_path = os.path.join(self.config.WEIGHTS_PATH, filename)
        self.model.save_weights(full_path)


# train/trainer.py에 추가
class DQNIQNTrainer:
    def __init__(self, env, config, agent, agent_name =None):
        self.env = env
        self.config = config
        self.agent = agent
        self.agent_name = agent_name
        self.model = agent.model
        self.target_model = agent.target_model
        self.optimizer = agent.optimizer
        self.discount_factor = agent.discount_factor
        self.epsilon = agent.epsilon
        self.epsilon_decay = agent.epsilon_decay
        self.epsilon_min = agent.epsilon_min
        self.train_step_counter = 0
        
        # IQN 특화 속성
        self.get_action_num_quantile = agent.get_action_num_quantile
        self.get_action_tau_min = agent.get_action_tau_min
        self.get_action_tau_max = agent.get_action_tau_max
        self.train_num_quantile = agent.train_num_quantile
        self.train_tau_min = agent.train_tau_min
        self.train_tau_max = agent.train_tau_max
        self.huber_threshold = agent.huber_threshold
        
        # 기타 초기화 변수
        self.best_reward = float('-inf')
        self.rewards = []
        self.episodes = []
        self.avg_reward = 0
        
        # 디렉토리 생성
        os.makedirs(self.config.WEIGHTS_PATH, exist_ok=True)
        os.makedirs(self.config.GRAPH_PATH, exist_ok=True)
        os.makedirs(self.config.LOG_PATH, exist_ok=True)
        
        # TensorBoard 설정
        self._setup_tensorboard()

    def _setup_tensorboard(self):
        """TensorBoard 설정"""
        current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
        log_base_dir = os.path.join(self.config.LOG_PATH, 'DQN')
        self.log_dir = os.path.join(log_base_dir, current_time)
        
        # 하위 디렉토리 생성
        os.makedirs(os.path.join(self.log_dir, 'training'), exist_ok=True)
        os.makedirs(os.path.join(self.log_dir, 'episode'), exist_ok=True)
        os.makedirs(os.path.join(self.log_dir, 'model'), exist_ok=True)
        os.makedirs(os.path.join(self.log_dir, 'evaluation'), exist_ok=True)
        
        # 요약 작성기 생성
        self.summary_writer = tf.summary.create_file_writer(self.log_dir)
        
        # 하이퍼파라미터 로깅
        self._log_hyperparameters()
        
        print(f"TensorBoard 로그가 저장될 위치: {self.log_dir}")
        print(f"로그를 보려면 다음을 실행하세요: tensorboard --logdir={log_base_dir}")

    def _log_hyperparameters(self):
        """하이퍼파라미터 로깅"""
        with self.summary_writer.as_default():
            hyperparams = (
                f"Model Type: IQN\n"
                f"Learning Rate: {self.config.LEARNING_RATE}\n"
                f"Epsilon Start: {self.epsilon}\n"
                f"Epsilon Decay: {self.epsilon_decay}\n"
                f"Epsilon Min: {self.epsilon_min}\n"
                f"Discount Factor: {self.discount_factor}\n"
                f"Batch Size: {self.config.BATCH_SIZE}\n"
                f"Memory Size: {self.config.MEMORY_SIZE}\n"
                f"IQN Embedding Dim: {self.config.IQN_EMBEDDING_DIM}\n"
                f"IQN Action Quantiles: {self.get_action_num_quantile}\n"
                f"IQN Action Tau Min: {self.get_action_tau_min}\n"
                f"IQN Action Tau Max: {self.get_action_tau_max}\n"
                f"IQN Train Quantiles: {self.train_num_quantile}\n"
                f"IQN Train Tau Min: {self.train_tau_min}\n"
                f"IQN Train Tau Max: {self.train_tau_max}\n"
                f"IQN Huber Threshold: {self.huber_threshold}\n"
            )
            tf.summary.text('hyperparameters', hyperparams, step=0)
            
            # 모델 아키텍처 로깅
            try:
                model_summary = []
                self.model.summary(print_fn=lambda x: model_summary.append(x))
                tf.summary.text('model_architecture', '\n'.join(model_summary), step=0)
            except ValueError as e:
                print(f"모델 아키텍처 로깅 건너뜀: {e}")
                tf.summary.text('model_architecture', "모델이 아직 빌드되지 않았습니다.", step=0)
            
            self.summary_writer.flush()

    def log_histogram_weights(self, episode):
        """모델 가중치 히스토그램 로깅"""
        if episode % 50 == 0:
            with self.summary_writer.as_default():
                for i, layer in enumerate(self.model.layers):
                    for weight in layer.weights:
                        tf.summary.histogram(
                            f"weights/{layer.name}/{weight.name}", 
                            weight, 
                            step=episode
                        )
                self.summary_writer.flush()

    def get_action(self, state):
        """행동 선택"""
        return self.agent.get_action(state)

    def get_memory_size(self):
        """메모리 사이즈 반환"""
        if hasattr(self.agent, 'memory'):
            if hasattr(self.agent.memory, 'tree'):
                return self.agent.memory.tree.n_entries
            elif hasattr(self.agent.memory, 'buffer'):
                return len(self.agent.memory.buffer)
            else:
                return len(self.agent.memory)
        return 0

    def update_target_network(self):
        """타깃 네트워크 업데이트"""
        self.agent.update_target_model()

    def _save_weights(self, best=False):
        """모델 가중치 저장"""
        prefix = "best_" if best else ""
        agent_type = self.agent_name
        
        os.makedirs(self.config.WEIGHTS_PATH, exist_ok=True)
        print(f"Current working directory: {os.getcwd()}")
        print(f"Saving weights to: {os.path.abspath(self.config.WEIGHTS_PATH)}")
        
        self.model.save_weights(f"{self.config.WEIGHTS_PATH}{prefix}{agent_type}_model.h5")

    def train(self):
        """IQN 학습 수행"""
        for ep in range(self.config.MAX_EPISODES):
            state = self.env.reset()
            state = np.reshape(state, [1, len(self.env.observation_space)])
            episode_reward, done = 0, False
            
            while not done:
                action = self.get_action(state)
                print('action:::::::::::::', action)
                next_state, reward, done = self.env.step(action)
                next_state = np.reshape(next_state, [1, len(self.env.observation_space)])
                
                episode_reward += reward
                
                # 경험 저장
                self.agent.append_sample(state[0], action, reward, next_state[0], done)
                
                # 학습 부분
                memory_size = self.get_memory_size()
                train_ready = memory_size >= self.config.TRAIN_START
                
                if train_ready:
                    loss = self.agent.train_model()
                    if loss is not None:
                        with self.summary_writer.as_default():
                            tf.summary.scalar('training/loss', loss, step=self.train_step_counter)
                            self.train_step_counter += 1
                
                state = next_state
                
                if done:
                    # 모델 가중치 히스토그램 로깅
                    if ep % 50 == 0:
                        self.log_histogram_weights(ep)
                    
                    # 타깃 네트워크 업데이트
                    if memory_size >= self.config.TRAIN_START and ep % 1 == 0:
                        self.update_target_network()
                    
                    # 평균 보상 계산
                    self.avg_reward = 0.9 * self.avg_reward + 0.1 * episode_reward if self.avg_reward != 0 else episode_reward
                    
                    # 메모리 크기 확인
                    memory_size = self.get_memory_size()
                    
                    # Rear vehicle character
                    if hasattr(self.env, 'rear_collision_mode'):
                        if self.env.rear_collision_mode == 1:
                            print("Rear vehicle: Unattentive Vehicle(Red)")
                        else:
                            print("Rear vehicle: Normal vehicle(Blue)")
                    
                    # 에피소드 정보 출력
                    print(f"episode: {ep:3d} | "
                          f"episode reward: {episode_reward:3.0f} | "
                          f"average reward: {self.avg_reward:3.2f} | "
                          f"memory length: {memory_size:4d} | "
                          f"epsilon: {self.epsilon:.4f}")
                    
                    # 에피소드 메트릭스 로깅
                    with self.summary_writer.as_default():
                        # Rewards
                        tf.summary.scalar('evaluation/reward', episode_reward, step=ep)
                        tf.summary.scalar('evaluation/avg_reward', self.avg_reward, step=ep)
                        
                        # Memory usage
                        tf.summary.scalar('memory/total_size', memory_size, step=ep)
                        
                        # Training parameters
                        tf.summary.scalar('training/epsilon', self.epsilon, step=ep)
                        
                        # Environment metrics
                        if hasattr(self.env, 'collision_num'):
                            tf.summary.scalar('evaluation/collisions', self.env.collision_num, step=ep)
                            
                            if hasattr(self.env, 'space1_count'):
                                tf.summary.scalar('environment/space1_count', self.env.space1_count, step=ep)
                                tf.summary.scalar('environment/space2_count', self.env.space2_count, step=ep)
                                tf.summary.scalar('environment/space3_count', self.env.space3_count, step=ep)
                                tf.summary.scalar('environment/space4_count', self.env.space4_count, step=ep)
                            
                            if hasattr(self.env, 'R1_count'):
                                tf.summary.scalar('environment/R1_count', self.env.R1_count, step=ep)
                                tf.summary.scalar('environment/R2_count', self.env.R2_count, step=ep)
                                tf.summary.scalar('environment/L1_count', self.env.L1_count, step=ep)
                                tf.summary.scalar('environment/L2_count', self.env.L2_count, step=ep)
                    
                    # 가중치 저장
                    if self.avg_reward > self.best_reward:
                        self.best_reward = self.avg_reward
                        self._save_weights(best=True)
                    
                    # 주기적인 모델 저장
                    if ep % self.config.SAVE_INTERVAL == 0:
                        self._save_weights(best=False)
                    
                    # 입실론 값 업데이트
                    if hasattr(self.agent, 'end_episode'):
                        self.epsilon = self.agent.end_episode(episode_reward)
                    
                    # TensorBoard 요약 업데이트
                    if ep % 10 == 0 and not ep == 0:
                        self.summary_writer.flush()
                    
                    # 종료 조건 확인
                    if self.avg_reward > self.config.REWARD_THRESHOLD:
                        print("\nProblem solved!")
                        self._save_weights(best=True)
                        break
            
            # 에피소드 종료 후 혼동 행렬 수집
            if hasattr(self.env, 'confusion_matrix_data_collection'):
                self.env.confusion_matrix_data_collection(ep)
            
            # 환경 종료
            if hasattr(self.env, 'end'):
                self.env.end()