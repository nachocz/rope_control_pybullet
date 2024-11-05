import pybullet as p
import pybullet_data
import yaml
import time
import numpy as np

# Load parameters from YAML file
with open("config.yaml", "r") as config_file:
    config = yaml.safe_load(config_file)

# Extract parameters from config
robot_params = config["robot"]
rope_params = config["rope"]
sim_params = config["simulation"]

# Connect to PyBullet
physicsClient = p.connect(p.GUI)

# Set the additional search path for URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Reset simulation and set gravity
p.resetSimulation()
p.setGravity(*sim_params["gravity"])

# Load the ground plane
planeId = p.loadURDF("plane.urdf")

# Load the Panda robot arm in a new favorable configuration away from the base
robot_start_pos = robot_params["start_position"]
robot_start_ori = p.getQuaternionFromEuler(robot_params["start_orientation"])
robotId = p.loadURDF(
    "franka_panda/panda.urdf",
    basePosition=robot_start_pos,
    baseOrientation=robot_start_ori,
    useFixedBase=True,
    flags=p.URDF_USE_SELF_COLLISION,
)

# Initialize robot's joints to a favorable starting position
initial_joint_positions = robot_params["initial_joint_positions"]
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

# Set a starting position for the first rope segment directly below but offset from the robot's end-effector
start_pos = rope_params["start_position"]

# Create rope segments
for i in range(num_segments):
    pos = [start_pos[0], start_pos[1], start_pos[2] - i * segment_length]

    # Create capsule segment
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
        basePosition=pos,
    )

    # Disable collisions between adjacent segments for stability
    if i > 0:
        p.setCollisionFilterPair(
            segment_id, rope_segments[i - 1], -1, -1, enableCollision=0
        )
    rope_segments.append(segment_id)

# Add constraints between rope segments
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

# Grasp the first rope segment with the robot's gripper
end_effector_index = 11
close_gripper(robotId)

# Attach the rope to the robot's end-effector
p.createConstraint(
    robotId,
    end_effector_index,
    rope_segments[0],
    -1,
    p.JOINT_FIXED,
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, -segment_length / 2],
)

# Set a fixed extended target position for the end-effector
targetPos = [
    0.8,
    0.0,
    1.0,
]  # Fixed position to keep the end-effector extended away from the base
jointPoses = p.calculateInverseKinematics(robotId, end_effector_index, targetPos)

# Set the robot joints to the calculated positions initially
control_force = 150  # Reduced control force for smoother motion
for i in range(7):
    p.setJointMotorControl2(
        robotId, i, p.POSITION_CONTROL, jointPoses[i], force=control_force
    )

# Simulation loop without dynamic motion to test the initial configuration
t = 0
dt = sim_params["time_step"]
simulation_duration = sim_params["duration"]

while t < simulation_duration:
    t += dt

    # Keep the joints in the calculated position to avoid unwanted movement
    for i in range(7):
        p.setJointMotorControl2(
            robotId, i, p.POSITION_CONTROL, jointPoses[i], force=control_force
        )

    # Step the simulation
    p.stepSimulation()
    time.sleep(dt)

# Disconnect from simulation
p.disconnect()
