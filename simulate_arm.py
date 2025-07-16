# simulate_arm.py
import mujoco
import mujoco.viewer
import numpy as np
import time

# Define the MuJoCo XML model for the planar arm
# This XML describes a 2-DOF planar robotic arm with two links and two revolute joints.
# It includes:
# - A 'world' body as the base.
# - A 'link1' body connected to the world via a 'hinge' joint (j1) for theta1.
# - A 'link2' body connected to 'link1' via another 'hinge' joint (j2) for theta2.
# - Geoms (capsules) to visualize the links, stretching 1m along the x-axis.
# - Actuators to control the position of each joint.
xml_model = """
<mujoco model="planar_arm">
    <compiler angle="radian" inertiafromgeom="true"/>
    <default>
        <joint armature="0.01" damping="0.1" limited="true"/>
        <geom contype="1" conaffinity="1" rgba="0.8 0.6 0.4 1"/>
    </default>

    <asset>
        <texture name="skybox" type="skybox" builtin="gradient" rgb1="0.4 0.5 0.6" rgb2="0 0 0" width="512" height="512"/>
        <material name="mat_plane" reflectance="0.5" shininess="0.5" specular="1" texrepeat="1 1" texture="skybox"/>
    </asset>

    <worldbody>
        <light directional="true" ambient="0.2 0.2 0.2" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1" pos="0 0 5"/>
        <geom name="floor" type="plane" size="0 0 0.1" material="mat_plane"/>

        <body name="link1" pos="0 0 0.1">
            <joint name="j1" type="hinge" pos="0 0 0" axis="0 0 1" range="-3.14 3.14"/>
            <!-- link1_geom: capsule type, radius 0.05m, half-length 0.5m (total 1m).
                 pos="0.5 0 0" places its center at 0.5m along x from link1's origin.
                 euler="0 90 0" rotates the capsule to align with the x-axis (default is z-axis). -->
            <geom name="link1_geom" type="capsule" size="0.05 0.5" pos="0.5 0 0" euler="0 90 0" rgba="0.1 0.5 0.8 1"/>
            <site name="tip1" pos="1 0 0" size="0.01" rgba="1 0 0 1"/>

            <!-- link2 body's position is 1m along x from link1's origin,
                 making its base coincide with the tip of link1. -->
            <body name="link2" pos="1 0 0">
                <joint name="j2" type="hinge" pos="0 0 0" axis="0 0 1" range="-3.14 3.14"/>
                <!-- link2_geom: same capsule properties as link1_geom. -->
                <geom name="link2_geom" type="capsule" size="0.05 0.5" pos="0.5 0 0" euler="0 90 0" rgba="0.8 0.1 0.5 1"/>
                <site name="tip2" pos="1 0 0" size="0.01" rgba="0 1 0 1"/>
            </body>
        </body>
    </worldbody>

    <actuator>
        <!-- Position actuators to control the joint angles -->
        <position name="a1" joint="j1" kp="100"/>
        <position name="a2" joint="j2" kp="100"/>
    </actuator>
</mujoco>
"""

# Create a MuJoCo model from the XML string
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

# Simulation parameters
duration = 10.0  # seconds
dt = model.opt.timestep # Simulation timestep
sim_time = 0.0

# Desired angular speeds for each joint (radians per second)
# These values determine how fast each arm segment will spin.
angular_speed_j1 = 0.5 * np.pi # 0.5 pi rad/s (90 degrees/s)
angular_speed_j2 = 1.0 * np.pi # 1.0 pi rad/s (180 degrees/s)

print(f"Starting planar arm simulation for {duration} seconds using standalone MuJoCo binding.")
print(f"Joint 1 angular speed: {angular_speed_j1:.2f} rad/s")
print(f"Joint 2 angular speed: {angular_speed_j2:.2f} rad/s")

# Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Simulation loop
    while sim_time < duration:
        # Calculate desired joint positions based on angular speeds and current time
        # The target position for each joint is its initial position plus (angular_speed * sim_time).
        # This creates a continuous rotation.
        target_j1_pos = angular_speed_j1 * sim_time
        target_j2_pos = angular_speed_j2 * sim_time

        # Set the control inputs for the actuators
        # data.ctrl is an array where each element corresponds to an actuator.
        # We are using position actuators, so we set the target position directly.
        data.ctrl[0] = target_j1_pos
        data.ctrl[1] = target_j2_pos

        # Step the simulation forward by one timestep
        mujoco.mj_step(model, data)

        # Increment simulation time
        sim_time += dt

        # Update the viewer
        viewer.sync()

        # Add a small delay to slow down the visualization for better observation
        time.sleep(dt) # Sleep for the simulation timestep duration

print("Simulation finished.")
