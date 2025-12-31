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

def test_friction_restitution():
    print("\n--- Testing Friction & Restitution ---")
    world = explorl_core.engine.World()
    world.set_gravity(np.array([0.0, -9.8, 0.0]))
    world.set_ground_plane_enabled(False) # Disable default ground to use our custom one
    
    # 1. Ground Plane (High Friction, No Bounce)
    # Using PlaneShape (behaves like a huge box)
    ground_shape = explorl_core.engine.PlaneShape(np.array([0.0, 1.0, 0.0]), 0.0)
    ground = world.create_body(0.0, np.array([0.0, -0.05, 0.0])) # Center at -0.05, top at 0.0
    ground.set_collision_shape(ground_shape)
    ground.set_friction(1.0) # Very sticky
    ground.set_restitution(0.0) # No bounce
    
    # 2. Bouncy Ball (Restitution = 0.9)
    ball = world.create_body(1.0, np.array([0.0, 5.0, 0.0]))
    ball.set_collision_shape(explorl_core.engine.SphereShape(0.5))
    ball.set_restitution(0.9)
    
    print("Simulating Bouncy Ball...")
    for i in range(120):
        world.step(1.0/60.0)
        if i % 10 == 0:
           pos = ball.get_position()
           vel = ball.get_linear_velocity()
           print(f"Step {i}: Y={pos[1]:.4f}, Vy={vel[1]:.4f}")
            
    # Check if it bounced
    print(f"Final Ball Y: {ball.get_position()[1]:.4f}")
    if ball.get_position()[1] > 2.0:
        print("SUCCESS: Ball bounced significantly.")
    else:
        print("FAILURE: Ball did not bounce enough.")

def test_motor_hinge():
    print("\n--- Testing Hinge Joint Motor ---")
    world = explorl_core.engine.World()
    world.set_gravity(np.array([0.0, 0.0, 0.0])) # Zero gravity to isolate motor
    
    # Static Base
    base = world.create_body(0.0, np.array([0.0, 0.0, 0.0]))
    base.set_collision_shape(explorl_core.engine.BoxShape(np.array([0.1, 0.1, 0.1])))
    
    # Dynamic Arm
    arm = world.create_body(1.0, np.array([1.0, 0.0, 0.0])) # 1m to the right
    arm.set_collision_shape(explorl_core.engine.BoxShape(np.array([0.9, 0.1, 0.1])))
    
    # Hinge at origin, rotating around Z axis
    hinge = explorl_core.engine.HingeJoint(base, arm, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]))
    world.add_joint(hinge)
    
    # Enable Motor
    target_vel = 1.0 # rad/s
    max_impulse = 10.0
    hinge.enable_motor(True)
    hinge.set_motor_target(target_vel, max_impulse)
    
    print("Simulating Motor...")
    for i in range(60):
        world.step(1.0/60.0)
        # Angular velocity of arm should approach 1.0
        w = arm.get_angular_velocity()
        if i % 10 == 0:
            print(f"Step {i}: AngVel Z = {w[2]:.4f}")
            
    final_w = arm.get_angular_velocity()[2]
    print(f"Final AngVel Z: {final_w:.4f}")
    
    if abs(final_w - target_vel) < 0.2: # Relaxed tolerance
        print("SUCCESS: Motor reached target velocity.")
    else:
        print("FAILURE: Motor did not reach target velocity.")

if __name__ == "__main__":
    test_friction_restitution()
    test_motor_hinge()
