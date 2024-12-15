python
Copy code
import tensorflow as tf
from tensorflow.keras import layers
import numpy as np

class DDPGAgent:
    def __init__(self, action_space, observation_space):
        self.action_space = action_space
        self.observation_space = observation_space
        self.actor = self.build_actor()
        self.critic = self.build_critic()

    def build_actor(self):
        model = tf.keras.Sequential([
            layers.InputLayer(input_shape=(self.observation_space.shape[0],)),
            layers.Dense(256, activation='relu'),
            layers.Dense(256, activation='relu'),
            layers.Dense(self.action_space.n, activation='tanh')
        ])
        return model

    def build_critic(self):
        model = tf.keras.Sequential([
            layers.InputLayer(input_shape=(self.observation_space.shape[0] + self.action_space.n,)),
            layers.Dense(256, activation='relu'),
            layers.Dense(256, activation='relu'),
            layers.Dense(1)
        ])
        return model

    def act(self, state):
        action = self.actor(state)
        return np.clip(action.numpy(), -1, 1)  # Ensure action is within valid bounds

    def train(self, state, action, reward, next_state, done):
        # Update actor and critic using the DDPG algorithm
        pass  # Placeholder for actual training code
