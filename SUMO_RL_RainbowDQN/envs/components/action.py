# envs/components/action.py
import numpy as np
import gym

class Action:
    def __init__(self, action_space):
        self.action_space = action_space
        
        # Handle both discrete and continuous action spaces
        if isinstance(action_space, gym.spaces.Discrete):
            self.is_discrete = True
            self.action_dim = 1
            self.action_bound = action_space.n - 1  # Maximum action value
        else:  # Continuous action space
            self.is_discrete = False
            self.action_dim = action_space.shape[0]
            self.action_bound = action_space.high[0]

    def process(self, action):
        """Process the action based on the action space type."""
        if self.is_discrete:
            # For discrete actions, ensure it's an integer within bounds
            return int(np.clip(action, 0, self.action_bound))
        else:
            # For continuous actions, clip to the valid range
            return np.clip(action, -self.action_bound, self.action_bound)