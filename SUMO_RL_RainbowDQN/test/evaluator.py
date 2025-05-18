# test/evaluator.py
import tensorflow as tf
import numpy as np

class DQNEvaluator:
    def __init__(self, env, trainer, config):
        self.env = env
        self.trainer = trainer
        self.config = config
         # 모델 빌드를 위해 더미 입력으로 한 번 호출
        dummy_state = np.zeros([1, self.env.observation_space.shape[0]])
        self.trainer.model(dummy_state)
        # 최적 가중치 로드 시도c
        try:
            self.trainer.model.load_weights(f"{self.config.WEIGHTS_PATH}best_sumo_dqn_model.h5")
            print("Successfully loaded best model weights")
        except:
            print("Could not load best weights, trying latest weights...")
            try:
                self.trainer.model.load_weights(f"{self.config.WEIGHTS_PATH}sumo_dqn_model.h5")
                print("Successfully loaded latest weights")
            except:
                print("Could not load any weights. Using initialized weights.")

    def evaluate(self, num_episodes=10, render=False):
        rewards_list = []
        steps_list = []
        
        for episode in range(num_episodes):
            print(f"\nEvaluating Episode {episode + 1}/{num_episodes}")
            
            state = self.env.reset()
            state = np.reshape(state, [1, self.env.observation_space.shape[0]])
            total_reward = 0
            steps = 0
            done = False
            
            while not done:
                if render:
                    # self.env.render()  # render_mode는 환경 생성 시에만 지정
                    self.env.gui_on=True
                
                q_value = self.trainer.model(tf.convert_to_tensor(state, dtype=tf.float32))
                action = np.argmax(q_value[0])
                next_state, reward, done = self.env.step(action)  # truncated 추가
                next_state = np.reshape(next_state, [1, self.env.observation_space.shape[0]])
                
                total_reward += reward
                steps += 1
                state = next_state
                
                if done:  
                    break
            self.env.end()
            rewards_list.append(total_reward)
            steps_list.append(steps)
            
            print(f"Episode {episode + 1} completed in {steps} steps with total reward {total_reward:.2f}")
            
        
        # Calculate statistics
        evaluation_stats = {
            'num_episodes': num_episodes,
            'mean_reward': np.mean(rewards_list),
            'std_reward': np.std(rewards_list),
            'min_reward': np.min(rewards_list),
            'max_reward': np.max(rewards_list),
            'mean_steps': np.mean(steps_list),
            'std_steps': np.std(steps_list),
            'rewards_list': rewards_list,
            'steps_list': steps_list
        }
        
        # Print summary statistics
        print("\nEvaluation Summary:")
        print(f"Mean Reward: {evaluation_stats['mean_reward']:.2f} ± {evaluation_stats['std_reward']:.2f}")
        print(f"Mean Steps: {evaluation_stats['mean_steps']:.2f} ± {evaluation_stats['std_steps']:.2f}")
        print(f"Min Reward: {evaluation_stats['min_reward']:.2f}")
        print(f"Max Reward: {evaluation_stats['max_reward']:.2f}")
        
        return evaluation_stats