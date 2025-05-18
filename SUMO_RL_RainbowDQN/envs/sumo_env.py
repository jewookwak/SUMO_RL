# envs/cart_pole_env.py
from envs.base_env import BaseEnv
from envs.components.state import State
from envs.components.action import Action
from envs.components.reward import Reward
from envs.sumo_origin_env2 import rlEnv

class SumoEnv(BaseEnv):
    def __init__(self):
        super().__init__()
        # Create CartPole-v1 environment
        self.env = rlEnv()
        self.env = testEnv()
        
        # Initialize handlers
        self.state_handler = State(self.env.observation_space)
        self.action_handler = Action(self.env.action_space)
        self.reward_handler = Reward()
        
        # Environment info
        self.observation_space = self.env.observation_space
        self.action_space = self.env.action_space
        
        # Episode tracking
        self.current_episode_steps = 0
        
    def reset(self):
        """Reset the environment and return the initial state."""
        self.current_episode_steps = 0
        state = self.env.reset()
        return self.state_handler.process(state)
        
    def step(self, action):
        """Execute action and return next_state, reward, done, info."""
        processed_action = self.action_handler.process(action)
        if len(self.env.step(processed_action))>=3:
            next_state, reward, done, info = self.env.step(processed_action)
        else:
            next_state, reward, done  = self.env.step(processed_action)
        
        self.current_episode_steps += 1
        
        processed_state = self.state_handler.process(next_state)
        processed_reward = self.reward_handler.process(
            reward, 
            done, 
            self.current_episode_steps
        )
            
        return processed_state, processed_reward, done, info
        
    def render(self):
        """Render the environment."""
        return self.env.render()
        
    def close(self):
        """Close the environment."""
        self.env.close()