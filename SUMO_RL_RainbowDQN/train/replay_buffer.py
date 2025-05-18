# replay_buffer.py
import numpy as np
import random
from collections import deque

class ReplayBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)
    
    def add_buffer(self, state, action, reward, next_state, done):
        experience = (state, action, reward, next_state, done)
        self.buffer.append(experience)
    
    def buffer_count(self):
        return len(self.buffer)
    
    def sample_batch(self, batch_size):
        if self.buffer_count() < batch_size:
            batch = random.sample(self.buffer, self.buffer_count())
        else:
            batch = random.sample(self.buffer, batch_size)
        
        states = np.array([sample[0][0] for sample in batch])
        actions = np.array([sample[1] for sample in batch])
        rewards = np.array([sample[2] for sample in batch])
        next_states = np.array([sample[3][0] for sample in batch])
        dones = np.array([sample[4] for sample in batch])
        
        return states, actions, rewards, next_states, dones

class SumTree:
    def __init__(self, capacity):
        self.capacity = capacity
        self.tree = np.zeros(2 * capacity - 1)
        self.data = np.zeros(capacity, dtype=object)
        self.n_entries = 0
        self.write = 0

    def _propagate(self, idx, change):
        parent = (idx - 1) // 2
        self.tree[parent] += change
        if parent != 0:
            self._propagate(parent, change)

    def _retrieve(self, idx, s):
        left = 2 * idx + 1
        right = left + 1
        if left >= len(self.tree):
            return idx
        if s <= self.tree[left]:
            return self._retrieve(left, s)
        else:
            return self._retrieve(right, s - self.tree[left])

    def total(self):
        return self.tree[0]

    def add(self, p, data):
        idx = self.write + self.capacity - 1
        self.data[self.write] = data
        self.update(idx, p)
        self.write += 1
        if self.write >= self.capacity:
            self.write = 0
        if self.n_entries < self.capacity:
            self.n_entries += 1

    def update(self, idx, p):
        change = p - self.tree[idx]
        self.tree[idx] = p
        self._propagate(idx, change)

    def get(self, s):
        idx = self._retrieve(0, s)
        dataIdx = idx - self.capacity + 1
        return idx, self.tree[idx], self.data[dataIdx]

class PrioritizedReplayBuffer:
    def __init__(self, capacity, alpha=0.6, beta=0.4, beta_increment=0.001, epsilon=0.01):
        """
        PrioritizedReplayBuffer 클래스 초기화
        
        Args:
            capacity (int): 버퍼 용량
            alpha (float): 우선순위 지수 (0=균등 샘플링, 1=완전 우선순위 샘플링)
            beta (float): 중요도 샘플링 지수 (0.4에서 시작하여 1로 증가)
            beta_increment (float): 에피소드마다 beta 증분
            epsilon (float): 우선순위에 더해지는 작은 값 (0으로 나누기 방지)
        """
        self.tree = SumTree(capacity)
        self.capacity = capacity
        self.alpha = alpha
        self.beta = beta
        self.beta_increment = beta_increment
        self.epsilon = epsilon
        self.max_priority = 1.0

    def reset(self):
        """버퍼 초기화"""
        self.tree = SumTree(self.capacity)
        self.max_priority = 1.0

    def _get_priority(self, error):
        """TD 오류로부터 우선순위 계산"""
        return (np.abs(error) + self.epsilon) ** self.alpha

    def add(self, error, sample):
        """
        샘플 추가 - 처음에는 최대 우선순위 사용
        
        Args:
            error (float): TD 오류
            sample (tuple): (state, action, reward, next_state, done) 형태의 경험
        """
        p = self._get_priority(error)
        self.max_priority = max(self.max_priority, p)
        self.tree.add(p, sample)

    def sample(self, n):
        """
        우선순위 기반 샘플링
        
        Args:
            n (int): 샘플링할 배치 크기
            
        Returns:
            list: 경험 샘플 배치
            list: 샘플링된 인덱스 목록
            ndarray: 중요도 샘플링 가중치
        """
        batch = []
        idxs = []
        segment = self.tree.total() / n
        priorities = []
        
        # beta 값 업데이트 (점점 1에 가까워지도록)
        self.beta = np.min([1., self.beta + self.beta_increment])
        
        # 샘플링
        for i in range(n):
            a = segment * i
            b = segment * (i + 1)
            
            s = random.uniform(a, b)
            (idx, p, data) = self.tree.get(s)
            priorities.append(p)
            batch.append(data)
            idxs.append(idx)
        
        # 중요도 샘플링 가중치 계산
        sampling_probabilities = np.array(priorities) / self.tree.total()
        is_weight = np.power(self.tree.n_entries * sampling_probabilities, -self.beta)
        is_weight /= is_weight.max()
        
        return batch, idxs, is_weight

    def update(self, idx, error):
        """
        우선순위 업데이트
        
        Args:
            idx (int): 업데이트할 샘플의 인덱스
            error (float): 새로운 TD 오류
        """
        p = self._get_priority(error)
        self.max_priority = max(self.max_priority, p)
        self.tree.update(idx, p)

class NStepMemory:
    def __init__(self, maxlen=5):
        self.maxlen = maxlen
        self.state = deque(maxlen=int(maxlen))
        self.action = deque(maxlen=int(maxlen))
        self.reward = deque(maxlen=int(maxlen))
        self.next_state = deque(maxlen=int(maxlen))
        self.done = deque(maxlen=int(maxlen))

    def append(self, state, next_state, reward, done, action):
        self.state.append(state)
        self.action.append(action)
        self.reward.append(reward)
        self.next_state.append(next_state)
        self.done.append(done)

    def sample(self):
        if len(self.state) == int(self.maxlen):
            done = [self.done[i] for i in range(self.maxlen-1)]
            if True in done:
                return None
            return {'state': list(self.state),
                    'next_state': list(self.next_state),
                    'reward': list(self.reward),
                    'done': list(self.done),
                    'action': list(self.action)}
        else:
            return None
    
    def clear(self):
        self.state.clear()
        self.next_state.clear()
        self.action.clear()
        self.reward.clear()
        self.done.clear()