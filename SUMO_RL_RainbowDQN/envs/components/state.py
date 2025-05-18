# envs/components/state.py
import numpy as np

class State:
    def __init__(self, observation_space):
        self.observation_space = observation_space
        print(observation_space.shape[0])
        self.state_dim = observation_space.shape[0]

    def process(self, state):
        return np.array(state, dtype=np.float32)
