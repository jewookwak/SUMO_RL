# test/evaluator.py
import tensorflow as tf
import numpy as np
import os
class Tester:
    def __init__(self, env, trainer, config, weight_path, agent=None):
        self.env = env
        self.trainer = trainer
        self.config = config
        self.weight_path = weight_path
        self.state_size = len(env.observation_space)
        self.sequence_length = config.PREV_SEQUENCE_LENGTH
            
        # TensorBoard 로깅을 위한 summary_writer 추가
        self.summary_writer = tf.summary.create_file_writer(
            os.path.join(self.config.RESULTS_PATH, 'evaluation'))
         # 모델 빌드를 위해 더미 입력으로 한 번 호출
        # dummy_state = np.zeros([1, self.state_size])
        dummy_state = np.zeros([1, self.sequence_length, self.state_size]) 
        self.trainer.model(dummy_state)
        # 최적 가중치 로드 시도
        full_path = os.path.join(self.config.WEIGHTS_PATH, weight_path)
        self.trainer.model.load_weights(full_path)
        self.avg_reward = 0
    def get_action(self, state_memory):
        """Choose action using epsilon-greedy policy"""
        if np.all(state_memory[0] == 5):  # Check if state memory has data
            return 5
        else:
            state = np.expand_dims(state_memory, axis=0)
            q_value = self.trainer.model(state)
            action = np.argmax(q_value[0])
            return action
    def evaluate(self, num_episodes=10, render=False):

        for episode in range(num_episodes):
            print(f"\nEvaluating Episode {episode + 1}/{num_episodes}")
            
            state = self.env.reset()
            state = np.reshape(state, [1, self.state_size])
            episode_reward = 0
            reward_memory = 0
            done = False
            state_memory = np.ones((self.sequence_length - 1, self.state_size))*5
            next_state_memory = np.ones((self.sequence_length - 2, self.state_size))*5
            next_state_memory = np.append(next_state_memory, state, axis=0)
            last_action = None
            while not done:
                if render:
                    self.env.gui_on=True
                if round(self.env.get_simulation_time(), 2).is_integer():
                    # Take action in environment
                    action = self.get_action(state_memory)
                    next_state, reward, done = self.env.step(action)
                    reward_memory += reward
                    
                    # Process next state
                    next_state = np.reshape(next_state, [1, self.state_size])
                    
                    # Update next state memory
                    if len(next_state_memory) > self.sequence_length - 1:
                        next_state_memory = np.delete(next_state_memory, 0, axis=0)
                    next_state_memory = np.append(next_state_memory, next_state, axis=0)
                    
                    episode_reward += reward_memory

                    # Update state
                    state = next_state
                    
                    # Update state memory
                    if len(state_memory) > self.sequence_length - 1:
                        state_memory = np.delete(state_memory, 0, axis=0)
                    state_memory = np.append(state_memory, state, axis=0)
                    
                    last_action = action
                    reward_memory = 0
                else:
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
                        episode_reward += reward_memory
                    self.avg_reward = 0.9 * self.avg_reward + 0.1 * episode_reward if self.avg_reward != 0 else episode_reward
                    print(f"Episode: {episode:3d} | "
                            f"Episode reward: {episode_reward:3.0f} | "
                            f"Average reward: {self.avg_reward:3.2f} | ")
                    print(f"Episode: {episode} | Collision: {self.env.collision_num} | "
                        f"Space sizes: 1st: {self.env.space1_count}, 2nd: {self.env.space2_count}, "
                        f"3rd: {self.env.space3_count}, 4th: {self.env.space4_count}")
                    
                    print(f"Episode: {episode} | Collision: {self.env.collision_num} | "
                        f"Space locations: R1: {self.env.R1_count}, R2: {self.env.R2_count}, "
                        f"L1: {self.env.L1_count}, L2: {self.env.L2_count}")
                    
                    # Log comprehensive episode metrics to TensorBoard
                    with self.summary_writer.as_default():
                        # Rewards
                        tf.summary.scalar('evaluation/reward', episode_reward, step=episode)
                        tf.summary.scalar('evaluation/avg_reward', self.avg_reward, step=episode)
                        

                        if hasattr(self.env, 'collision_num'):
                            tf.summary.scalar('evaluation/collisions', self.env.collision_num, step=episode)
                            
                            # Space size metrics
                            tf.summary.scalar('environment/space1_count', self.env.space1_count, step=episode)
                            tf.summary.scalar('environment/space2_count', self.env.space2_count, step=episode)
                            tf.summary.scalar('environment/space3_count', self.env.space3_count, step=episode)
                            tf.summary.scalar('environment/space4_count', self.env.space4_count, step=episode)
                            
                            # Space location metrics
                            tf.summary.scalar('environment/R1_count', self.env.R1_count, step=episode)
                            tf.summary.scalar('environment/R2_count', self.env.R2_count, step=episode)
                            tf.summary.scalar('environment/L1_count', self.env.L1_count, step=episode)
                            tf.summary.scalar('environment/L2_count', self.env.L2_count, step=episode)
                    self.env.end()

        return 