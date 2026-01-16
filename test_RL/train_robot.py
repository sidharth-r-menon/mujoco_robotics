import gymnasium as gym
from stable_baselines3 import PPO
import os

# 1. Create the Environment
# "Reacher-v4" is a standard MuJoCo environment.
# render_mode="human" lets you SEE it (slows down training). 
# For pure training, use render_mode=None.
env = gym.make("Reacher-v4", render_mode="human")

# 2. Define the RL Model (The "Brain")
# We use PPO (Proximal Policy Optimization).
# "MlpPolicy" means we use a standard Dense Neural Network (Multi-Layer Perceptron).
# verbose=1 prints progress to the console.
model = PPO("MlpPolicy", env, verbose=1)

print("---------------------------------------")
print("STARTING TRAINING... (Look at the arm struggle!)")
print("---------------------------------------")

# 3. Train the Agent
# We train for 10,000 timesteps. In real research, this is millions.
model.learn(total_timesteps=10000)
model.save("ppo_reacher")

print("---------------------------------------")
print("TRAINING FINISHED. NOW TESTING THE BRAIN.")
print("---------------------------------------")

# 4. Test the Trained Agent
# We reset the environment to start fresh.
obs, info = env.reset()

for _ in range(1000):
    # The model predicts the best action (joint torques) based on observation
    action, _states = model.predict(obs, deterministic=True)
    
    # Apply the action to the robot
    obs, reward, terminated, truncated, info = env.step(action)
    
    # If the task is done, reset
    if terminated or truncated:
        obs, info = env.reset()

env.close()