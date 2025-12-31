import sys
import os
import time
import math
import numpy as np

# Add project root to path to allow importing explorl package
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import explorl
from explorl.envs import G1Env

class G1WalkerPolicy:
    """
    A simple procedural gait controller (Central Pattern Generator) for G1.
    This demonstrates walking capability without requiring full RL training.
    """
    def __init__(self, env):
        self.env = env
        self.joint_names = env.joint_names
        
        # Identify Joint Indices
        self.left_hip_pitch_idx = self._find_idx("left_hip_pitch_joint")
        self.left_knee_idx = self._find_idx("left_knee_joint")
        self.left_ankle_pitch_idx = self._find_idx("left_ankle_pitch_joint")
        
        self.right_hip_pitch_idx = self._find_idx("right_hip_pitch_joint")
        self.right_knee_idx = self._find_idx("right_knee_joint")
        self.right_ankle_pitch_idx = self._find_idx("right_ankle_pitch_joint")
        
        # Identify Arm Joints to keep them steady
        self.arm_indices = [i for i, name in enumerate(self.joint_names) if "shoulder" in name or "elbow" in name]
        
        # Gait Parameters
        self.freq = 0.8 # Slower, more stable walk
        self.amplitude_hip = 0.5 # Larger steps
        self.amplitude_knee = 0.8
        self.phase_offset = np.pi # Alternating legs
        
        # Base Offsets (Standing Pose)
        self.offset_hip = 0.05 # Lean forward slightly to walk forward
        self.offset_knee = 0.6
        self.offset_ankle = -0.45 # Adjust for new hip offset and box foot
        
        # Arm Swing Parameters
        self.arm_swing_amp = 0.8 
        
    def _find_idx(self, substring):
        for i, name in enumerate(self.joint_names):
            if substring in name:
                return i
        # Fallback for approximate matching
        for i, name in enumerate(self.joint_names):
            parts = substring.replace("_joint", "").split("_")
            if all(p in name for p in parts):
                return i
        return -1

    def get_action(self, t):
        action = np.zeros(self.env.num_joints)
        
        # Default Pose for all joints
        # Arms naturally down
        for idx in self.arm_indices:
            action[idx] = 0.0
            
        # Time-based Phase
        phase_l = t * 2 * np.pi * self.freq
        phase_r = phase_l + self.phase_offset
        
        # --- Left Leg ---
        if self.left_hip_pitch_idx >= 0:
            # Hip swings back and forth
            # sin(phase) > 0: Forward Swing
            # sin(phase) < 0: Backward Stance (Push)
            action[self.left_hip_pitch_idx] = self.offset_hip + self.amplitude_hip * np.sin(phase_l)
            
        if self.left_knee_idx >= 0:
            # Knee bends during swing phase (when hip is moving forward)
            # Peak bend should be at mid-swing (phase ~ pi/2)
            # sin(phase) matches this.
            # We want to keep leg straight during stance (sin < 0).
            swing_bend = max(0, np.sin(phase_l)) 
            knee_val = self.offset_knee + self.amplitude_knee * swing_bend
            action[self.left_knee_idx] = knee_val
            
        if self.left_ankle_pitch_idx >= 0:
            # Ankle Strategy:
            # 1. Swing: Lift toe to clear ground (Dorsiflexion) -> Negative target (relative to default)
            # 2. Stance: Push off (Plantarflexion) -> Positive target
            # Heuristic: Counter-act knee/hip but add push-off
            
            base_ankle = self.offset_ankle - 0.5 * (action[self.left_hip_pitch_idx] - self.offset_hip)
            
            # Add push-off at end of stance (phase ~ 2pi or 0)
            # Stance phase is sin < 0 (pi to 2pi).
            # We want peak push-off near 2pi (Toe-off).
            # cos(phase) peaks at 0/2pi.
            push_off = 0.3 * max(0, np.cos(phase_l)) 
            
            action[self.left_ankle_pitch_idx] = base_ankle + push_off
            
        # --- Right Leg ---
        if self.right_hip_pitch_idx >= 0:
            action[self.right_hip_pitch_idx] = self.offset_hip + self.amplitude_hip * np.sin(phase_r)
            
        if self.right_knee_idx >= 0:
            swing_bend = max(0, np.sin(phase_r))
            knee_val = self.offset_knee + self.amplitude_knee * swing_bend
            action[self.right_knee_idx] = knee_val
            
        if self.right_ankle_pitch_idx >= 0:
            base_ankle = self.offset_ankle - 0.5 * (action[self.right_hip_pitch_idx] - self.offset_hip)
            push_off = 0.3 * max(0, np.cos(phase_r))
            action[self.right_ankle_pitch_idx] = base_ankle + push_off
            
        # --- Arm Swing (Counter-balance) ---
        # Left Arm swings with Right Leg
        left_shoulder_idx = self._find_idx("left_shoulder_pitch")
        right_shoulder_idx = self._find_idx("right_shoulder_pitch")
        
        if left_shoulder_idx >= 0:
             action[left_shoulder_idx] = self.arm_swing_amp * np.sin(phase_r)
        if right_shoulder_idx >= 0:
             action[right_shoulder_idx] = self.arm_swing_amp * np.sin(phase_l)

        return action

def main():
    print("Initializing G1 Environment...")
    env = G1Env(render_mode="human")
    
    print("Initializing Procedural Walker Policy...")
    policy = G1WalkerPolicy(env)
    
    # Increase PD gains for stable walking
    policy.kp = 80.0
    policy.kd = 3.0
    
    # Enable recording
    if env.renderer:
        env.renderer.start_recording("g1_walking.mp4")
    
    obs, info = env.reset()
    
    t = 0.0
    # Sync with render FPS (60Hz) not physics FPS (240Hz)
    dt = 1.0 / 60.0 
    last_time = time.time()
    
    print("Starting Simulation...")
    print("Press ESC in the window to exit.")
    
    try:
        while True:
            # Sync to real-time (approximate)
            # time.sleep(dt) 
            
            # Policy
            target_q = policy.get_action(t)
            
            # Step
            obs, reward, done, truncated, info = env.step(target_q)
            
            # Print status
            if env.pelvis:
                pos = env.pelvis.get_position()
                vel = env.pelvis.get_linear_velocity()
                print(f"\rTime: {t:.2f} | Pelvis Z: {pos[2]:.2f} | VX: {vel[0]:.2f} | FPS: {1.0/(time.time()-last_time) if time.time()>last_time else 0:.1f}", end="")
            
            last_time = time.time()
            t += dt
            
            # Check window close
            if env.renderer and env.renderer.should_close():
                 break
            
            # Auto-stop after 10 seconds for testing
            if t > 10.0:
                print("\nAuto-stopping after 10s...")
                break
            
            if done or truncated:
                print("\nResetting environment...")
                obs, info = env.reset()
                t = 0.0
                # policy.reset()

    except KeyboardInterrupt:
        pass
    finally:
        print("\nClosing environment...")
        if env.renderer:
            env.renderer.stop_recording()
        env.close()

if __name__ == "__main__":
    main()
