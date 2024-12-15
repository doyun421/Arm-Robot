 강화학습 환경 설정 (OpenAI Gym)
python
Copy code
import gym
from gym import spaces
import numpy as np

class RobotArmEnv(gym.Env):
    def __init__(self):
        super(RobotArmEnv, self).__init__()
        # Action space: 4 discrete actions (e.g., move up, down, left, right)
        self.action_space = spaces.Discrete(4)
        # Observation space: 6 continuous values (e.g., joint angles, position)
        self.observation_space = spaces.Box(low=np.array([0, 0, 0, 0, 0, 0]), high=np.array([1, 1, 1, 1, 1, 1]), dtype=np.float32)
        
    def reset(self):
        # Initialize robot arm's state (e.g., joint positions)
        return np.random.rand(6)
    
    def step(self, action):
        # Define how the environment responds to actions (robot movement)
        state = np.random.rand(6)  # Update the state
        reward = -np.sum(state)  # Minimize the error, maximize the reward
        done = False
        info = {}
        return state, reward, done, info

    def render(self):
        pass
