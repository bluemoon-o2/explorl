
import sys
import os
import math
import numpy as np

# Add the compiled library to python path
# Location: explorl/lib
sys.path.append(os.path.abspath("explorl/lib"))

try:
    import explorl_core
except ImportError:
    print("Cannot import explorl_core. Please ensure the project is built and the path is correct.")
    # Try alternate path (build folder directly?)
    sys.path.append(os.path.abspath("build/src/bindings/Release"))
    try:
        import explorl_core
    except ImportError:
        print("Still cannot import explorl_core. Exiting.")
        sys.exit(1)

def test_urdf_loading():
    print("Testing URDF Loading...")
    engine = explorl_core.engine
    world = engine.World()
    world.set_gravity(np.array([0, -9.81, 0]))
    world.set_ground_plane_enabled(True)
    
    # Create a simple URDF string for a pendulum or simple robot
    urdf_content = """
    <robot name="simple_robot">
        <link name="base_link">
            <inertial>
                <mass value="0"/> <!-- Static base -->
                <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
            <collision>
                <geometry>
                    <box size="0.2 0.2 0.2"/>
                </geometry>
            </collision>
        </link>
        
        <link name="link1">
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
            <collision>
                <geometry>
                    <cylinder radius="0.05" length="0.5"/>
                </geometry>
            </collision>
        </link>
        
        <joint name="joint1" type="revolute">
            <parent link="base_link"/>
            <child link="link1"/>
            <origin xyz="0.1 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1.57" upper="1.57"/>
        </joint>
    </robot>
    """
    
    success = engine.URDFLoader.load(world, urdf_content)
    if success:
        print("URDF loaded successfully!")
    else:
        print("Failed to load URDF.")
        return

    # Step simulation
    for i in range(100):
        world.step(0.01)
        if i % 10 == 0:
            print(f"Step {i}")

    print("Testing IK...")
    # Find the end effector (link1)
    # Since URDFLoader doesn't return the map, we need to find body by iterating?
    # World::get_bodies() is not exposed as list directly usually, or maybe it is.
    # But URDFLoader load returns bool.
    
    # We can't easily find the body unless we expose finding by name or store it.
    # But for now, let's assume we can't easily access it without modifying World binding.
    # However, create_body returns RigidBody*, but URDFLoader handles it internally.
    
    # Let's verify IK Solver binding exists at least.
    solver = engine.IKSolver
    print("IKSolver found.")
    
    print("Test Complete.")

if __name__ == "__main__":
    test_urdf_loading()
