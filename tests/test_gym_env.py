import sys
import os
import numpy as np

# Setup path to package root
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from explorl.envs.cartpole import CartPoleEnv
import time

def test_cartpole():
    print("Testing CartPole Environment with C++ Physics Backend...")
    
    # Initialize Environment
    # env = CartPoleEnv()
    # Use Software Backend for rendering
    env = CartPoleEnv(render_mode="human")
    
    # Run for multiple episodes to test stability
    for episode in range(5):
        obs, info = env.reset(seed=42 + episode)
        print(f"--- Episode {episode} ---")
        
        start_time = time.time()
        steps = 0
        
        done = False
        while not done:
            # Render the current state
            env.render()
            
            # Random Action: 0 or 1
            action = np.random.randint(0, env.action_space_n)
            
            # Step
            obs, reward, terminated, truncated, info = env.step(action)
            
            steps += 1
            
            if terminated or truncated:
                done = True
                
            # Optional: Print debug info every 50 steps
            if steps % 50 == 0:
                print(f"Step {steps}")

        end_time = time.time()
        duration = end_time - start_time
        print(f"Episode Finished. Steps: {steps}, FPS: {steps / duration:.2f}")
            
    env.close()

if __name__ == "__main__":
    test_cartpole()
