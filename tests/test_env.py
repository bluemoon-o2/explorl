import sys
import os
import time
import numpy as np

# Add the library path
# The pyd is in explorl/lib
lib_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../explorl/lib"))
sys.path.append(lib_path)

print(f"Adding path: {lib_path}")

try:
    import explorl_core
    print("Successfully imported explorl_core!")
except ImportError as e:
    print(f"Failed to import explorl_core: {e}")
    sys.exit(1)

def test_cartpole():
    print("\n--- Testing CartPoleSim ---")
    sim = explorl_core.envs.CartPoleSim()
    sim.reset(42.0)
    
    # Check zero-copy state access
    state = sim.get_state()
    print(f"Initial State (numpy): {state}")
    print(f"State type: {type(state)}")
    
    # Verify it is a numpy array
    if not isinstance(state, np.ndarray):
        print("ERROR: State is not a numpy array!")
        return

    # Verify modification reflects (if it was a view, but get_state might return a new view each time? 
    # Nanobind usually returns a view if configured)
    # But wait, get_state returns a view of the internal array.
    
    print("Stepping environment...")
    sim.step(1)
    print(f"State after step: {state}") # If it's a view, this might update?
    
    # Check if the python object 'state' updated or if we need to call get_state() again
    # This depends on if nanobind returns a view of the C++ memory.
    # The C++ code uses: nb::ndarray<nb::numpy, double, nb::shape<4>>(sim.get_state_ptr(), { 4 })
    # This should be a view.
    
    state_new = sim.get_state()
    print(f"State from new call: {state_new}")
    
    if np.array_equal(state, state_new):
         print("Zero-Copy View Confirmation: The previous 'state' object updated automatically (or matches).")
    else:
         print("Note: Previous 'state' object did not update. This might be a copy.")

    # Render test
    # renderer = explorl_core.envs.EnvRenderer(800, 600, "Python CartPole", True)
    # renderer.render(sim)
    # time.sleep(1)
    # renderer.close()

def test_pendulum():
    print("\n--- Testing PendulumSim ---")
    sim = explorl_core.envs.PendulumSim()
    sim.reset(42.0)
    state = sim.get_state()
    print(f"Initial Pendulum State: {state}")
    
    sim.step(0.0)
    print(f"State after step: {state}")

if __name__ == "__main__":
    test_cartpole()
    test_pendulum()
