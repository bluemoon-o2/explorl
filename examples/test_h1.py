import sys
import os
import time
import math
import numpy as np

# Add project root to path to allow importing explorl package
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Use the package import
import explorl

def main():
    # Create World
    world = explorl.engine.World()
    # Default Z-up gravity
    world.set_gravity(np.array([0, 0, -9.81]))
    
    # Disable default ground and create a custom visible Box ground
    world.set_ground_plane_enabled(False)
    
    # Create a large box for the ground (100x100x1), placed so top surface is at z=0
    ground_half_extents = np.array([50.0, 50.0, 0.5])
    ground_pos = np.array([0.0, 0.0, -0.5])
    
    ground = world.create_body(0.0, ground_pos)
    ground.set_name("ground")
    ground.set_collision_shape(explorl.engine.BoxShape(ground_half_extents))
    ground.set_visual_color([0.2, 0.25, 0.2]) # Dark Greenish Grey
    ground.set_friction(2.0)
    ground.set_restitution(0.0)

    # Load H1 Robot URDF
    urdf_path = os.path.join(os.path.dirname(__file__), '../unitree_ros/robots/h1_description/urdf/h1.urdf')
    urdf_path = os.path.abspath(urdf_path)
    
    print(f"Loading URDF from: {urdf_path}")
    
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found at {urdf_path}")
        return

    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    
    # Pass True for z_up_to_y_up to fix orientation
    if not explorl.engine.URDFLoader.load(world, urdf_content):
        print("Failed to load URDF")
        return

    print("URDF loaded successfully.")
    
    # Setup Bodies (Colors and Initial Pose)
    bodies = world.get_bodies()
    print(f"Number of bodies: {len(bodies)}")
    
    # Find pelvis for camera tracking
    pelvis_body = None
    for body in bodies:
        if "pelvis" in body.get_name():
            pelvis_body = body
            break

    # 2. Setup Properties
    for body in bodies:
        name = body.get_name()
        
        # Increase friction for ALL parts to prevent sliding when lying down
        body.set_friction(1.0)
        
        # NOTE: The URDF defines all materials as dark gray (0.1, 0.1, 0.1).
        # We manually set metallic colors here for better visualization.
        if "pelvis" in name or "trunk" in name:
            body.set_visual_color([0.7, 0.7, 0.7]) # Metallic Grey
        elif "left" in name:
            body.set_visual_color([0.6, 0.6, 0.65]) # Slightly bluish grey
        elif "right" in name:
            body.set_visual_color([0.6, 0.6, 0.65]) # Slightly bluish grey
        else:
            body.set_visual_color([0.7, 0.7, 0.7]) # Metallic Grey
        
        # Increase friction for feet to prevent sliding
        if "ankle" in name:
            body.set_friction(4.0)

    # Setup Joints (PD Control)
    joints = world.get_joints()
    print(f"Number of joints: {len(joints)}")
    
    # Nominal Standing Pose (Approximate)
    # Hip Pitch: -0.4, Knee: 0.8, Ankle: -0.4
    standing_pose = {
        "hip_pitch": -0.2,
        "knee": 0.4,
        "ankle": -0.3
    }
    
    # Enable motors for holding position
    # PyBullet typically uses high gains. H1 is heavy.
    kp = 1000.0 # Proportional gain (Stiffer)
    kd = 30.0   # Derivative gain (More damping)
    
    for joint in joints:
        # Check if it's a HingeJoint (simple check by attribute presence)
        if hasattr(joint, "enable_motor"):
            joint.enable_motor(True)
            
            joint_name = joint.get_name()
            target_angle = 0.0
            
            # Simple heuristic to set target angles based on name
            for key, val in standing_pose.items():
                if key in joint_name:
                    target_angle = val
                    # Mirror for other side if needed (usually handled by URDF axis, but let's be simple)
                    break
            
            joint.set_motor_target(target_angle, kp, kd)

    # Simulation Loop
    print("Starting simulation loop...")
    # Initialize Renderer
    from explorl.utils import SimpleRenderer
    renderer = SimpleRenderer(title="H1 Test")
    
    # Start Recording (Disabled for performance)
    print("Recording video to h1_test.mp4... (Disabled for performance)")
    renderer.start_recording("h1_test.mp4", fps=60)
    
    # Set camera to follow pelvis
    if pelvis_body:
        print("Camera following pelvis")
        # Offset in Z-up: X=2, Y=2, Z=1
        renderer.set_target_body(pelvis_body, offset=[2.0, 2.0, 1.0])
    else:
        renderer.set_camera_position(np.array([2.0, 2.0, 1.5]))
        renderer.set_camera_target(np.array([0, 0, 0.8]))
    
    sim_renderer = explorl.render.WorldRenderer(world)
    
    dt = 1.0 / 240.0
    t = 0.0
    
    # Run for 10 seconds
    start_time = time.time()
    frame_count = 0
    
    # Render at 60 FPS
    render_dt = 1.0 / 60.0
    steps_per_frame = int(render_dt / dt)
    
    # Debug Ankle Height
    l_ankle = None
    for body in bodies:
        if "left_ankle_link" in body.get_name():
            l_ankle = body
            break

    while not renderer.should_close():
        
        # Step physics multiple times
        for _ in range(steps_per_frame):
            # Manual Damping to prevent sliding/jitter
            for body in bodies:
                lin_vel = body.get_linear_velocity()
                ang_vel = body.get_angular_velocity()
                # Apply opposing force proportional to velocity
                body.apply_force_central(-0.5 * lin_vel) 
                body.apply_torque(-0.1 * ang_vel)

            world.step(dt)
            t += dt

            # Debug Print (every 60 steps = ~0.25s)
            if l_ankle and int(t / dt) % 60 == 0:
                pos = l_ankle.get_position()
                # print(f"L_Ankle Z: {pos[2]:.4f}") 

            
        # Render
        renderer.render(sim_renderer)
        
        # Sync with real time
        frame_count += 1
        elapsed = time.time() - start_time
        expected = frame_count * render_dt
        if expected > elapsed:
            time.sleep(expected - elapsed)
            
    print("Simulation finished.")

if __name__ == "__main__":
    main()
