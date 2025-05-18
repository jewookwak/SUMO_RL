# envs/base_env.py
from abc import ABC, abstractmethod
import gym

class BaseEnv(ABC):
    def __init__(self):
        self.env = None
        
    @abstractmethod
    def reset(self):
        pass
    
    @abstractmethod
    def step(self, action):
        pass
    
    @abstractmethod
    def close(self):
        pass