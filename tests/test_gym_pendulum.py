import sys
import os

# Add project root to path
sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.getcwd(), "explorl", "lib"))
sys.path.append(os.path.join(os.getcwd(), "explorl", "lib", "Release"))

import explorl # This sets up paths via __init__.py
try:
    import explorl_core as core
except ImportError as e:
    print(f"Error importing explorl_core: {e}")
    print(f"sys.path: {sys.path}")
    # Try finding the file manually to debug
    lib_dir = os.path.join(os.getcwd(), "explorl", "lib")
    if os.path.exists(lib_dir):
        print(f"Contents of {lib_dir}: {os.listdir(lib_dir)}")
    sys.exit(1)

import time
import math
import numpy as np

def test_pendulum():
    print("Initializing Pendulum Environment via Python Bindings...")
    
    # Create Renderer (Hardware or Software)
    renderer = core.envs.EnvRenderer(800, 600, "Pendulum-v1 (Python)", False)
    
    # Create Simulation
    env = core.envs.PendulumSim()
    env.reset(42.0)
    
    print("Starting Loop...")
    
    for i in range(500):
        if renderer.should_close():
            break
            
        # Get state
        state = env.get_state() # [cos, sin, theta_dot]
        theta = math.atan2(state[1], state[0])
        w = state[2]
        
        # Simple policy: Swing up logic (Bang-Bang)
        # If w is small and theta is near 0 (up), stabilize.
        # Otherwise, add energy.
        
        action = 0.0
        # Swing up controller
        # E = 0.5 * I * w^2 + mgl(1 - cos)
        # Target E = 2mgl
        # Here m=1, g=9.81, l=1 -> mgl ~ 9.81
        # Target E ~ 19.62
        # E_curr = 0.5 * 1/3 * w^2 + 9.81 * (1 - cos) ? No I=ml^2/12?
        # Actually just apply torque in direction of velocity if E < E_target
        # Or simpler: -2.0 * sign(w) if at bottom?
        
        # Just random action for visual test
        action = math.sin(i * 0.1) * 2.0
        reward = env.step(action)
        if i % 20 == 0:
            print(f"Step {i}: Reward={reward:.4f}, Theta={theta:.4f}, W={w:.4f}")
        renderer.render(env)
        
        time.sleep(0.016)
        
    renderer.close()
    print("Done.")

if __name__ == "__main__":
    test_pendulum()
