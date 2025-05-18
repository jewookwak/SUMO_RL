# train/trainer/LSTM_prev_nstep_trainer.py
import os
import numpy as np
import tensorflow as tf
from tensorflow.keras.optimizers import Adam
from datetime import datetime
from collections import deque
import random
from train.network.LSTM_networks import LSTMDQN
from train.replay_buffer import ReplayBuffer
import traci

class PREVNSTEPLSTMDQNTrainer:
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
        # 에피소드 카운터와 입실론 감소 주기 설정
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
            # 기존 LSTM 모델 초기화
            self.model = LSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
            self.target_model = LSTMDQN(self.state_size, self.action_size, self.batch_size, self.sequence_length)
            self.optimizer = Adam(learning_rate=self.config.LEARNING_RATE)
            
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
        
        # TensorBoard setup
        self._setup_tensorboard()

        # Initialize target network
        self.update_target_network()


    def update_target_network(self):
        """Update target network with weights from main network"""
        if self.using_external_agent:
            self.agent.update_target_model()
        else:
            self.target_model.set_weights(self.model.get_weights())
            
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
    
    def train_step(self, states, actions, rewards, next_states, dones):
        """Perform a single training step on a batch of experiences"""
        model_params = self.model.trainable_variables
        with tf.GradientTape() as tape:
            # Current state Q-values
            predicts = self.model(states)
            one_hot_action = tf.one_hot(actions, self.action_size)
            predicts = tf.reduce_sum(one_hot_action * predicts, axis=1)

            # Next state Q-values from target network
            target_predicts = self.target_model(next_states)
            target_predicts = tf.stop_gradient(target_predicts)

            # Bellman equation to compute target Q-values
            max_q = np.amax(target_predicts, axis=-1)
            targets = rewards + (1 - dones) * self.discount_factor * max_q
            loss = tf.reduce_mean(tf.square(targets - predicts))

        # Compute gradients and update model
        grads = tape.gradient(loss, model_params)
        self.optimizer.apply_gradients(zip(grads, model_params))
        # 로그 기록 추가
        with self.summary_writer.as_default():
            tf.summary.scalar('training/loss', loss, step=self.train_step_counter)
            self.train_step_counter += 1


    def train(self):
        """Main training loop"""
        for ep in range(self.config.MAX_EPISODES):
            # Initialize episode variables
            state = self.env.reset()
            state = np.reshape(state, [1, self.state_size])
            episode_reward = 0
            reward_memory = 0
            done = False
            
            # Initialize state memories for sequence
            # state_memory = np.zeros((self.sequence_length - 1, self.state_size))
            # next_state_memory = np.zeros((self.sequence_length - 2, self.state_size))
            state_memory = np.ones((self.sequence_length - 1, self.state_size))*5
            next_state_memory = np.ones((self.sequence_length - 2, self.state_size))*5
            next_state_memory = np.append(next_state_memory, state, axis=0)

            
            last_action = None
            decision_frequency = 1  # Make decision every 1000ms
            
            # Episode loop
            while not done:
                # Check if it's time to make a decision
                if round(self.env.get_simulation_time(), 2).is_integer():
                    # Choose action based on agent type
                    if self.using_external_agent:
                        action = self.agent.get_action(state_memory)
                    else:
                        action = self.get_action(state_memory)

                    print('action:::::::: ', action)
                    # Take action in environment
                    next_state, reward, done = self.env.step(action)
                    reward_memory += reward
                    
                    # Process next state
                    next_state = np.reshape(next_state, [1, self.state_size])
                    
                    # Update next state memory
                    if len(next_state_memory) > self.sequence_length - 1:
                        next_state_memory = np.delete(next_state_memory, 0, axis=0)
                    next_state_memory = np.append(next_state_memory, next_state, axis=0)
                    
                    episode_reward += reward_memory
                    
                    # Store experience based on agent type -> storing memory per second
                    if not np.all(state_memory[0] == 0) and not np.all(state_memory[0] == 5):  # 시퀀스가 비어있지 않은지 확인
                        if self.using_external_agent:
                            self.agent.append_sample(state_memory, action, reward_memory, next_state_memory, done)
                        else:
                            self.append_sample(state_memory, action, reward_memory, next_state_memory, done)
                            self.consolidate_memory()
                    
                    # Train model based on agent type
                    if self.using_external_agent:
                        memory_size = (
                            self.agent.memory.tree.n_entries 
                            if hasattr(self.agent.memory, 'tree') 
                            else len(self.agent.memory.buffer) 
                            if hasattr(self.agent.memory, 'buffer') 
                            else 0
                        )
                        # print('memory_size: ',memory_size)
                        if memory_size >= self.config.TRAIN_START:
                            loss = self.agent.train_model()
                            with self.summary_writer.as_default():
                                tf.summary.scalar('training/loss', loss, step=self.train_step_counter)
                                self.train_step_counter += 1
                            

                    else:
                        if len(self.memory) >= self.config.TRAIN_START:
                            # Sample batch and train
                            mini_batch = random.sample(self.memory, self.batch_size)
                            
                            states = np.array([sample[0] for sample in mini_batch])
                            actions = np.array([sample[1] for sample in mini_batch])
                            rewards = np.array([sample[2] for sample in mini_batch])
                            next_states = np.array([sample[3] for sample in mini_batch])
                            dones = np.array([sample[4] for sample in mini_batch])
                            
                            self.train_step(states, actions, rewards, next_states, dones)
                            
                            # Log additional metrics (epsilon already logged in train_step)
                            # with self.summary_writer.as_default():
                            #     tf.summary.scalar('training/epsilon', self.epsilon, step=ep)
                            #     tf.summary.scalar('training/memory_size', len(self.memory), step=ep)

                    # Update state
                    state = next_state
                    
                    # Update state memory
                    if len(state_memory) > self.sequence_length - 1:
                        state_memory = np.delete(state_memory, 0, axis=0)
                    state_memory = np.append(state_memory, state, axis=0)
                    
                    last_action = action
                    reward_memory = 0
                else:
                    # Continue with last action between decision points
                    next_state, reward, done = self.env.step(last_action)
                    
                    if done:
                        next_state = np.reshape(next_state, [1, self.state_size])
                        # Update memories
                        if len(next_state_memory) > self.sequence_length - 1:
                            next_state_memory = np.delete(next_state_memory, 0, axis=0)
                        next_state_memory = np.append(next_state_memory, next_state, axis=0)
                        
                        state = next_state
                        
                        if len(state_memory) > self.sequence_length - 1:
                            state_memory = np.delete(state_memory, 0, axis=0)
                        state_memory = np.append(state_memory, state, axis=0)
                    
                    reward_memory += reward
                
                # Episode end processing
                if done:
                    if (reward_memory !=0): #rewardmemory가 0인 것은 앞에서 terminal reward가 메모리에 이미 담긴것으로 중복 처리 할 필요 없음
                        # 유효한 state 시퀀스가 있는지 확인
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
                    
                    # Compute metrics
                    self.avg_reward = 0.9 * self.avg_reward + 0.1 * episode_reward if self.avg_reward != 0 else episode_reward
                    
                    # Get memory size based on agent type
                    memory_size = (
                        self.agent.memory.tree.n_entries 
                        if self.using_external_agent and hasattr(self.agent.memory, 'tree')
                        else len(self.agent.memory.buffer) 
                        if self.using_external_agent and hasattr(self.agent.memory, 'buffer')
                        else len(self.memory) if not self.using_external_agent
                        else 0
                    )
                    # Update target network periodically
                    if memory_size >= self.config.TRAIN_START and ep % 1 == 0:
                        self.update_target_network()
                    # Rear vehicle character
                    if self.env.rear_collision_mode == 1:
                        print("Rear vehicle: Unattentive Vehicle(Red)")
                    else:
                        print("Rear vehicle: Normal vehicle(Blue)")
                    # Log episode metrics
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
                    
                    # Log comprehensive episode metrics to TensorBoard
                    with self.summary_writer.as_default():
                        # Rewards
                        tf.summary.scalar('evaluation/reward', episode_reward, step=ep)
                        tf.summary.scalar('evaluation/avg_reward', self.avg_reward, step=ep)
                        
                        # Memory usage
                        tf.summary.scalar('memory/total_size', memory_size, step=ep)
                        
                        if not self.using_external_agent:
                            tf.summary.scalar('memory/action0_size', len(self.memory0), step=ep) 
                            tf.summary.scalar('memory/action1_size', len(self.memory1), step=ep)
                            tf.summary.scalar('memory/action2_size', len(self.memory2), step=ep)
                            tf.summary.scalar('memory/action3_size', len(self.memory3), step=ep)
                            tf.summary.scalar('memory/action4_size', len(self.memory4), step=ep)
                            tf.summary.scalar('memory/action5_size', len(self.memory5), step=ep)
                        
                        # Training parameters
                        tf.summary.scalar('training/epsilon', self.epsilon, step=ep)
                        # tf.summary.scalar('training/memory_size', len(self.memory), step=ep)
                        # Environment metrics if available
                        if hasattr(self.env, 'collision_num'):
                            tf.summary.scalar('evaluation/collisions', self.env.collision_num, step=ep)
                            
                            # Space size metrics
                            tf.summary.scalar('environment/space1_count', self.env.space1_count, step=ep)
                            tf.summary.scalar('environment/space2_count', self.env.space2_count, step=ep)
                            tf.summary.scalar('environment/space3_count', self.env.space3_count, step=ep)
                            tf.summary.scalar('environment/space4_count', self.env.space4_count, step=ep)
                            
                            # Space location metrics
                            tf.summary.scalar('environment/R1_count', self.env.R1_count, step=ep)
                            tf.summary.scalar('environment/R2_count', self.env.R2_count, step=ep)
                            tf.summary.scalar('environment/L1_count', self.env.L1_count, step=ep)
                            tf.summary.scalar('environment/L2_count', self.env.L2_count, step=ep)
                    
                    # Save model if improved
                    if self.avg_reward > self.best_reward:
                        self.best_reward = self.avg_reward
                        self._save_weights(best=True)
                    
                    # Periodic model saving
                    if ep % self.config.SAVE_INTERVAL == 0:
                        self._save_weights(best=False)
                        
                    # Save confusion matrix if available
                    if hasattr(self.env, 'confusion_matrix_data_collection'):
                        self.env.confusion_matrix_data_collection(ep)

                    # End environment if needed
                    if hasattr(self.env, 'end'):
                        self.env.end()
                    
                    # Save TensorBoard summaries more frequently for important episodes
                    if ep % 10 == 0 and not ep == 0:
                        self.summary_writer.flush()  # Make sure all summaries are written to disk
                        
                    # Check for termination condition
                    if self.avg_reward > self.config.REWARD_THRESHOLD:
                        print("\nProblem solved!")
                        self._save_weights(best=True)
                        break

    def _save_weights(self, best=False):
        """Save model weights"""
        prefix = "best_" if best else ""
        agent_type = self.agent_name if self.using_external_agent else "prev_nstep_lstm_dqn"
        
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
        # agent_type = "LSTMDQN_External" if self.using_external_agent else "LSTMDQN"
        # log_base_dir = os.path.join(self.config.LOG_PATH, agent_type)
        # self.log_dir = os.path.join(log_base_dir, current_time)
        # Create log directory structure
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
            
    def get_simulation_time(self):
        """Helper method to get simulation time"""
        if hasattr(self.env, 'get_simulation_time'):
            return self.env.get_simulation_time()
        else:
            import traci
            return np.round(traci.simulation.getTime(), 2)