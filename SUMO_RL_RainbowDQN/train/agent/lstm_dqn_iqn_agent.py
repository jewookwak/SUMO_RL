# train/lstm_dqn_iqn_agent.py
import os
import numpy as np
import tensorflow as tf
from tensorflow.keras.layers import Dense, Input, LSTM, Conv1D, MaxPool1D, Concatenate, Lambda
from tensorflow.keras.optimizers import Adam
from train.replay_buffer import ReplayBuffer, PrioritizedReplayBuffer, NStepMemory
import random
from collections import deque


class LSTMDQNIQNAgent:
    """LSTM-DQN agent with Implicit Quantile Networks."""
    
    def __init__(self, state_size, action_size, config):
        """Initialize agent."""
        self.state_size = state_size
        self.action_size = action_size
        self.config = config
        
        # Initialize memory buffer
        self.memory = PrioritizedReplayBuffer(config.MEMORY_SIZE, 
                                              alpha=config.PER_ALPHA, 
                                              beta=config.PER_BETA, 
                                              beta_increment=config.PER_BETA_INCREMENT,
                                              epsilon=config.PER_EPSILON)
        self.n_step_memory = NStepMemory(maxlen=config.N_STEP)
        
        # Implicit Quantile Networks parameters
        self.embedding_dim = config.IQN_EMBEDDING_DIM
        self.action_quantiles = config.IQN_ACTION_QUANTILES
        self.action_tau_min = config.IQN_ACTION_TAU_MIN
        self.action_tau_max = config.IQN_ACTION_TAU_MAX
        self.train_quantiles = config.IQN_TRAIN_QUANTILES
        self.train_tau_min = config.IQN_TRAIN_TAU_MIN
        self.train_tau_max = config.IQN_TRAIN_TAU_MAX
        self.huber_threshold = config.IQN_HUBER_THRESHOLD
        
        # Sequence settings
        self.sequence_length = config.SEQUENCE_LENGTH
        self.batch_size = config.BATCH_SIZE
        
        # Training parameters
        self.epsilon = config.EPSILON
        self.epsilon_decay = config.EPSILON_DECAY
        self.epsilon_min = config.EPSILON_MIN
        self.discount_factor = config.DISCOUNT_FACTOR
        self.learning_rate = config.LEARNING_RATE
        self.n_step = config.N_STEP

        # 에피소드 카운터와 입실론 감소 주기 설정
        self.episode_counter = 0
        self.epsilon_update_frequency = 10  # 10 에피소드마다 입실론 감소
        
        # Create main and target networks
        from train.network.LSTM_networks import LSTMDQN
        self.model = self._build_iqn_model(state_size, action_size)
        self.target_model = self._build_iqn_model(state_size, action_size)
        
        # Optimizer
        self.optimizer = Adam(learning_rate=self.learning_rate, epsilon=1e-2/self.batch_size)
        
        # Initialize target model with same weights
        self.update_target_model()
        
    def _build_iqn_model(self, state_size, action_size):
        """
        Build and return an IQN model based on LSTM architecture
        """
        # Start with base LSTM model structure from LSTM_networks
        from train.network.LSTM_networks import LSTMDQN
        model = LSTMDQN(state_size, action_size, batch_size=self.batch_size, sequence_length=self.sequence_length)
        
        # We're using the model structure but with IQN functionality
        # This setup allows us to reuse the LSTM architecture from LSTMDQN while adding IQN capabilities
        # The actual IQN functionality is implemented in the train_model method
        
        return model
        
    def update_target_model(self):
        """Update target model with weights from main model."""
        self.target_model.set_weights(self.model.get_weights())
    
    def get_action(self, state_memory):
        """
        Choose action using epsilon-greedy policy with IQN
        
        Args:
            state: Current state batch with sequence history
            
        Returns:
            int: Selected action
        """
        if np.all(state_memory[0] == 0) or np.all(state_memory[0] == 5):  # 시퀀스가 비어있는지 확인
            return 5
        state = np.expand_dims(state_memory, axis=0)
        if np.random.rand() <= self.epsilon:
            return np.random.randint(self.action_size)
        else:
            # For IQN, we use a fixed number of quantiles for action selection
            # The IQN paper suggests using fewer quantiles for action selection than for training
            state_tensor = tf.convert_to_tensor(state, dtype=tf.float32)
            q_values = self.model(state_tensor)
            
            # Return action with highest mean Q-value
            # Ensure we return a Python int, not a numpy or tensor type
            return int(np.argmax(q_values[0]))
    
    def append_sample(self, state, action, reward, next_state, done):
        """
        Add experience to replay buffer
        
        Args:
            state: Current state with sequence history
            action: Action taken
            reward: Reward received
            next_state: Next state with sequence history
            done: Whether episode is done
        """
        # Add to n-step buffer
        self.n_step_memory.append(state, next_state, reward, done, action)
        
        # Process n-step returns if we have enough transitions
        n_step_sample = self.n_step_memory.sample()
        if n_step_sample:
            # Calculate n-step return
            n_step_reward = 0
            for i in range(self.n_step):
                if i < len(n_step_sample['reward']):
                    n_step_reward += self.discount_factor**i * n_step_sample['reward'][i]
            
            # Get the initial state and final next_state for n-step
            initial_state = n_step_sample['state'][0]
            final_next_state = n_step_sample['next_state'][-1]
            final_done = n_step_sample['done'][-1]
            action = n_step_sample['action'][0]
            
            # For prioritized replay, we need an initial estimate of the TD error
            # For IQN, we estimate this using the mean Q-values
            state_tensor = tf.convert_to_tensor(np.expand_dims(initial_state, axis=0), dtype=tf.float32)
            next_state_tensor = tf.convert_to_tensor(np.expand_dims(final_next_state, axis=0), dtype=tf.float32)
            
            current_q = self.model(state_tensor)[0][action]
            next_q = self.target_model(next_state_tensor)[0][np.argmax(self.model(next_state_tensor)[0])]
            
            if final_done:
                target = n_step_reward
            else:
                target = n_step_reward + self.discount_factor**self.n_step * next_q
                
            td_error = abs(current_q - target)
            
            # Add experience to prioritized replay buffer
            self.memory.add(td_error, (initial_state, action, n_step_reward, final_next_state, final_done))
    
    def train_model(self):
        """
        Train model using a batch from experience replay
        
        Returns:
            float: Loss value
        """
        if self.memory.tree.n_entries < self.batch_size:
            return 0.0
        
        # Sample batch and importance weights from PER
        batch, indices, is_weights = self.memory.sample(self.batch_size)
        
        states = np.array([sample[0] for sample in batch])
        actions = np.array([sample[1] for sample in batch])
        rewards = np.array([sample[2] for sample in batch])
        next_states = np.array([sample[3] for sample in batch])
        dones = np.array([sample[4] for sample in batch])
        
        # Convert to tensors
        states_tensor = tf.convert_to_tensor(states, dtype=tf.float32)
        next_states_tensor = tf.convert_to_tensor(next_states, dtype=tf.float32)
        rewards_tensor = tf.convert_to_tensor(rewards, dtype=tf.float32)
        actions_tensor = tf.convert_to_tensor(actions, dtype=tf.int32)
        dones_tensor = tf.convert_to_tensor(dones, dtype=tf.float32)
        is_weights_tensor = tf.convert_to_tensor(is_weights, dtype=tf.float32)
        
        # IQN specific loss calculation using quantile regression
        with tf.GradientTape() as tape:
            # Get current Q-values and next Q-values
            current_q = self.model(states_tensor)
            next_q = self.model(next_states_tensor)
            target_q = self.target_model(next_states_tensor)
            
            # Get best actions based on current network
            best_actions = tf.cast(tf.argmax(next_q, axis=1), tf.int32)  # Cast to int32
            
            # Gather Q-values for actions taken
            action_masks = tf.one_hot(actions_tensor, self.action_size)
            gathered_q = tf.reduce_sum(current_q * action_masks, axis=1)
            
            # Compute target for each batch element
            batch_idx = tf.range(self.batch_size, dtype=tf.int32)  # Specify dtype
            target_values = rewards_tensor + (1 - dones_tensor) * self.discount_factor * tf.gather_nd(
                target_q, 
                tf.stack([batch_idx, best_actions], axis=1)
            )
            
            # Compute quantile regression loss (Huber loss)
            td_errors = target_values[:, tf.newaxis] - gathered_q
            
            # Ensure consistent data types for huber loss calculation
            td_errors = tf.cast(td_errors, tf.float32)
            huber_threshold = tf.constant(self.huber_threshold, dtype=tf.float32)
            
            huber_loss = tf.where(
                tf.abs(td_errors) <= huber_threshold,
                0.5 * tf.square(td_errors),
                huber_threshold * (tf.abs(td_errors) - 0.5 * huber_threshold)
            )
            
            # Apply importance sampling weights from PER
            weighted_loss = huber_loss * is_weights_tensor[:, tf.newaxis]
            
            # Total loss
            loss = tf.reduce_mean(weighted_loss)
        
        # Compute gradients and update network
        gradients = tape.gradient(loss, self.model.trainable_variables)
        self.optimizer.apply_gradients(zip(gradients, self.model.trainable_variables))
        
        # Update priorities in replay buffer
        td_errors_np = td_errors.numpy()
        for i in range(self.batch_size):
            # Make sure we're using a scalar value for the error
            error_val = float(np.abs(td_errors_np[i][0]))
            self.memory.update(indices[i], error_val)
        
        return loss.numpy()
    
    def end_episode(self, episode_reward):
        """
        Perform end-of-episode operations
        
        Args:
            episode_reward: Total reward for the episode
            
        Returns:
            float: Updated epsilon value
        """
        self.episode_counter += 1
        # 10 에피소드마다 입실론 감소
        if self.episode_counter % self.epsilon_update_frequency == 0 and self.epsilon > self.epsilon_min:
            old_epsilon = self.epsilon
            self.epsilon *= self.epsilon_decay
            print(f"Epsilon decreased from {old_epsilon:.4f} to {self.epsilon:.4f}")
        
        
        return self.epsilon
    
    def save_model(self, path):
        """Save model weights to file."""
        if not os.path.exists(path):
            os.makedirs(path)
        self.model.save_weights(os.path.join(path, 'lstm_dqn_iqn_model.h5'))
        
    def load_model(self, path):
        """Load model weights from file."""
        self.model.load_weights(os.path.join(path, 'lstm_dqn_iqn_model.h5'))
        self.update_target_model()