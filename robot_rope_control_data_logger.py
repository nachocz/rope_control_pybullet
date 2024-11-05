import pybullet as p
import pybullet_data
import yaml
import time
import numpy as np
import csv

# Load parameters from YAML file
with open("config.yaml", "r") as config_file:
    config = yaml.safe_load(config_file)

# Extract parameters from config
robot_params = config["robot"]
rope_params = config["rope"]
sim_params = config["simulation"]
trajectory_params = config["trajectory"]

# Connect to PyBullet
physicsClient = p.connect(p.GUI)

# Set the additional search path for URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Reset simulation and set gravity
p.resetSimulation()
p.setGravity(*sim_params["gravity"])

# Load the ground plane
planeId = p.loadURDF("plane.urdf")

# Load the Panda robot arm in a favorable configuration away from the base
robot_start_pos = robot_params["start_position"]
robot_start_ori = p.getQuaternionFromEuler(robot_params["start_orientation"])
robotId = p.loadURDF(
    "franka_panda/panda.urdf",
    basePosition=robot_start_pos,
    baseOrientation=robot_start_ori,
    useFixedBase=True,
    flags=p.URDF_USE_SELF_COLLISION,
)

# Initialize robot's joints to a favorable starting position (bent configuration to avoid singularities)
initial_joint_positions = [
    0.0,
    -0.5,
    0.3,
    -1.0,
    0.0,
    1.0,
    0.785,
]  # Adjusted to keep the arm bent
for i in range(len(initial_joint_positions)):
    p.resetJointState(robotId, i, initial_joint_positions[i])

# Gripper control functions
finger_joint_indices = [9, 10]


def control_gripper(robot_id, opening_angle):
    for joint_index in finger_joint_indices:
        p.setJointMotorControl2(
            robot_id,
            joint_index,
            p.POSITION_CONTROL,
            targetPosition=opening_angle,
            force=10,
        )


def open_gripper(robot_id):
    control_gripper(robot_id, 0.04)  # Open position


def close_gripper(robot_id):
    control_gripper(robot_id, 0.0)  # Closed position


# Rope setup
rope_length = rope_params["length"]
num_segments = rope_params["num_segments"]
segment_length = rope_length / num_segments
segment_radius = rope_params["segment_radius"]
segment_mass = rope_params["segment_mass"]
rope_segments = []

# Set the starting position for the first rope segment directly below the robot's end-effector
targetPos = [0.5, 0.0, 0.6]  # Initial position for the end-effector (fixed height)
end_effector_index = 11
jointPoses = p.calculateInverseKinematics(robotId, end_effector_index, targetPos)

# Set the robot joints to the calculated positions initially
control_force = 150  # Reduced control force for smoother motion
for i in range(7):
    p.setJointMotorControl2(
        robotId, i, p.POSITION_CONTROL, jointPoses[i], force=control_force
    )

# Grasp the first rope segment with the robot's gripper
close_gripper(robotId)

# Create the rope segments in a hanging configuration
for i in range(num_segments):
    segment_pos = [targetPos[0], targetPos[1], targetPos[2] - (i + 1) * segment_length]
    segment_col_shape = p.createCollisionShape(
        shapeType=p.GEOM_CAPSULE, radius=segment_radius, height=segment_length
    )
    segment_vis_shape = p.createVisualShape(
        shapeType=p.GEOM_CAPSULE,
        radius=segment_radius,
        length=segment_length,
        rgbaColor=[1, 0, 0, 1],
    )
    segment_id = p.createMultiBody(
        baseMass=segment_mass,
        baseCollisionShapeIndex=segment_col_shape,
        baseVisualShapeIndex=segment_vis_shape,
        basePosition=segment_pos,
    )

    # Disable collisions between adjacent segments for stability
    if i > 0:
        p.setCollisionFilterPair(
            segment_id, rope_segments[i - 1], -1, -1, enableCollision=0
        )
    rope_segments.append(segment_id)

# Add constraints between rope segments to form a hanging rope
for i in range(num_segments - 1):
    p.createConstraint(
        parentBodyUniqueId=rope_segments[i],
        parentLinkIndex=-1,
        childBodyUniqueId=rope_segments[i + 1],
        childLinkIndex=-1,
        jointType=p.JOINT_POINT2POINT,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, segment_length / 2],
        childFramePosition=[0, 0, -segment_length / 2],
    )

# Attach the first rope segment to the robot's end-effector using a fixed constraint
end_effector_constraint = p.createConstraint(
    robotId,
    end_effector_index,
    rope_segments[0],
    -1,
    p.JOINT_FIXED,
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, -segment_length / 2],
)

# Create a ghost anchor point at the loose end of the rope
ghost_anchor_position = [
    targetPos[0],
    targetPos[1],
    targetPos[2] - (num_segments + 1) * segment_length,
]
ghost_anchor_id = p.createMultiBody(
    baseMass=0, baseCollisionShapeIndex=-1, basePosition=ghost_anchor_position
)

# Attach the loose end of the rope to the ghost anchor point
ghost_anchor_constraint = p.createConstraint(
    parentBodyUniqueId=rope_segments[-1],
    parentLinkIndex=-1,
    childBodyUniqueId=ghost_anchor_id,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, -segment_length / 2],
    childFramePosition=[0, 0, 0],
)

# Step the simulation for a few moments to stabilize the setup with the ghost anchor in place
for _ in range(1000):
    p.stepSimulation()
    time.sleep(sim_params["time_step"] / 10)  # Faster stepping to stabilize

# Remove the ghost anchor constraint to allow free hanging
p.removeConstraint(ghost_anchor_constraint)

# Simulation loop with gentle horizontal swinging motion to generate data
t = 0
dt = sim_params["time_step"]
simulation_duration = sim_params["duration"]

amplitude = trajectory_params["amplitude"]
frequency = trajectory_params["frequency"]

# Open CSV files for logging
with open("robot_states_log.csv", mode="w", newline="") as robot_log_file, open(
    "rope_states_log.csv", mode="w", newline=""
) as rope_log_file:

    robot_writer = csv.writer(robot_log_file)
    rope_writer = csv.writer(rope_log_file)

    # Headers for robot states log
    robot_headers = [
        "time",
        "end_effector_x",
        "end_effector_y",
        "end_effector_z",
        "end_effector_qx",
        "end_effector_qy",
        "end_effector_qz",
        "end_effector_qw",
        "end_effector_vx",
        "end_effector_vy",
        "end_effector_vz",
        "end_effector_angular_vx",
        "end_effector_angular_vy",
        "end_effector_angular_vz",
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
        "joint_7",
    ]
    robot_writer.writerow(robot_headers)

    # Headers for rope states log
    rope_headers = ["time"]
    for i in range(num_segments):
        rope_headers += [
            f"segment_{i}_x",
            f"segment_{i}_y",
            f"segment_{i}_z",
            f"segment_{i}_vx",
            f"segment_{i}_vy",
            f"segment_{i}_vz",
        ]
    rope_writer.writerow(rope_headers)

    while t < simulation_duration:
        t += dt

        # Calculate horizontal oscillating target position for the end-effector (gentle horizontal motion)
        dx = amplitude * np.sin(2 * np.pi * frequency * t)
        current_target_pos = [
            targetPos[0] + dx,
            targetPos[1],
            targetPos[2],
        ]  # Y and Z are kept fixed

        # Calculate the inverse kinematics for the current target position
        jointPoses = p.calculateInverseKinematics(
            robotId, end_effector_index, current_target_pos
        )

        # Apply joint positions
        for i in range(7):
            p.setJointMotorControl2(
                robotId, i, p.POSITION_CONTROL, jointPoses[i], force=control_force
            )

        # Step the simulation
        p.stepSimulation()

        # Get end-effector state
        link_state = p.getLinkState(
            robotId, end_effector_index, computeLinkVelocity=True
        )
        end_effector_pos = [round(x, 4) for x in link_state[0]]  # Position [x, y, z]
        end_effector_ori = [
            round(x, 4) for x in link_state[1]
        ]  # Orientation as quaternion [qx, qy, qz, qw]
        end_effector_lin_vel = [
            round(x, 4) for x in link_state[6]
        ]  # Linear velocity [vx, vy, vz]
        end_effector_ang_vel = [
            round(x, 4) for x in link_state[7]
        ]  # Angular velocity [angular_vx, angular_vy, angular_vz]

        # Collect joint angles
        joint_data = [round(x, 4) for x in jointPoses[:7]]

        # Write robot states to CSV
        robot_writer.writerow(
            [
                round(t, 4),
                *end_effector_pos,
                *end_effector_ori,
                *end_effector_lin_vel,
                *end_effector_ang_vel,
                *joint_data,
            ]
        )

        # Collect rope segment states (positions and velocities)
        rope_data = [round(t, 4)]
        for segment_id in rope_segments:
            seg_pos, _ = p.getBasePositionAndOrientation(segment_id)
            seg_lin_vel, _ = p.getBaseVelocity(segment_id)
            rope_data.extend([round(x, 4) for x in seg_pos])
            rope_data.extend([round(x, 4) for x in seg_lin_vel])

        # Write rope states to CSV
        rope_writer.writerow(rope_data)

        # Pause to match real-time
        time.sleep(dt)

# Disconnect from simulation
p.disconnect()
