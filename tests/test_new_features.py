import sys
import os
import numpy as np

# Setup path
lib_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../explorl/lib"))
sys.path.append(lib_path)

try:
    import explorl_core
except ImportError:
    lib_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../explorl/lib/Release"))
    sys.path.append(lib_path)
    import explorl_core

def test_ray_casting():
    print("\n--- Testing Ray Casting ---")
    world = explorl_core.engine.World()
    
    # 1. Sphere at (0, 5, 0) radius 1
    sphere = world.create_body(0.0, np.array([0.0, 5.0, 0.0]))
    sphere.set_collision_shape(explorl_core.engine.SphereShape(1.0))
    
    # Cast ray from (0, 10, 0) down to (0, 0, 0)
    hit = world.ray_test(np.array([0.0, 10.0, 0.0]), np.array([0.0, 0.0, 0.0]))
    
    if hit:
        print(f"Hit 1: Pos={hit['position']}, Normal={hit['normal']}, Fraction={hit['fraction']}")
        # Should hit top of sphere at (0, 6, 0)
        # Distance from (0,10,0) to (0,6,0) is 4. Total ray length is 10. Fraction = 0.4.
        if abs(hit['position'][1] - 6.0) < 0.01:
            print("SUCCESS: Hit sphere top correctly.")
        else:
            print("FAILURE: Hit position wrong.")
    else:
        print("FAILURE: Did not hit sphere.")

    # Cast ray that misses
    hit_miss = world.ray_test(np.array([5.0, 10.0, 0.0]), np.array([5.0, 0.0, 0.0]))
    if hit_miss:
        print("FAILURE: Hit something where nothing exists.")
    else:
        print("SUCCESS: Missed correctly.")

def test_rk4_integrator():
    print("\n--- Testing RK4 Integrator ---")
    world = explorl_core.engine.World()
    world.set_gravity(np.array([0.0, -9.8, 0.0]))
    world.set_integrator(1) # Enable RK4
    
    # Free falling body
    # y(t) = y0 - 0.5*g*t^2
    # y(1) = 10 - 4.9 = 5.1
    
    body = world.create_body(1.0, np.array([0.0, 10.0, 0.0]))
    
    print("Simulating (RK4)...")
    for i in range(60):
        world.step(1.0/60.0)
        
    pos = body.get_position()
    print(f"Final Pos (RK4): {pos}")
    
    if abs(pos[1] - 5.05) < 0.1: # Allow some numerical error but RK4 should be precise
        print("SUCCESS: RK4 gravity integration matches analytical.")
    else:
        print("FAILURE: RK4 integration diverged.")

if __name__ == "__main__":
    test_ray_casting()
    test_rk4_integrator()
