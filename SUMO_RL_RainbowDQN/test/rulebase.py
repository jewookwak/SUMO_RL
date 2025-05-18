# test/ruleebse.py
import tensorflow as tf
import numpy as np

class RuleBase:
    def __init__(self, env):
        self.env = env

    def simulate(self, num_episodes=10, render=False):
        LC_succeed_list = []
        LC_succeed_count=0
        steps_list = []
        
        for episode in range(num_episodes):
            print(f"\nSimulating Episode {episode + 1}/{num_episodes}")
            
            steps = 0
            done = False
            self.env.reset()
            while not done:
                # print('self.env.Anti_RVC_on: ',self.env.Anti_RVC_on)
                if self.env.Anti_RVC_on:                    
                    action = self.env.Closest_space()
                    _,_,done =self.env.step(action)
                else:
                    _,_,done = self.env.step(5)
                steps += 1                
                if done:  
                    break
            print(f"Episode {episode + 1} completed in {steps} steps and LC succeed is {self.env.ego_LC_completed}")
            self.env.end()
            LC_succeed_list.append(LC_succeed_count)
            steps_list.append(steps)
            
            
            
        
        # Calculate statistics
        simulation_stats = {
            'num_episodes': num_episodes,
            'num_LC_succeed_number' :self.env.LC_succeed_num,
            'LC_succeed_list' : LC_succeed_list,
            'mean_steps': np.mean(steps_list),
            'std_steps': np.std(steps_list),
            'steps_list': steps_list
        }
        
        # Print summary statistics
        print("\nSimulation Summary:")
        print(f"LC Succeed Numbers: {simulation_stats['num_LC_succeed_number']}")
        print(f"Mean Steps: {simulation_stats['mean_steps']:.2f} ± {simulation_stats['std_steps']:.2f}")

        return simulation_stats