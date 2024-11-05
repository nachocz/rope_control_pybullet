import pybullet as p
import pybullet_data

class RobotControl:
    def __init__(self, robot_params):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
        self.end_effector_index = 11

        # Set initial joint positions
        self.initial_joint_positions = robot_params["initial_joint_positions"]
        for i, pos in enumerate(self.initial_joint_positions):
            p.resetJointState(self.robot_id, i, pos)

    def move_end_effector_position(self, target_position):
        """
        Moves the end effector to a specified position using position control.

        Args:
        - target_position (list): The target (x, y, z) coordinates for the end effector.
        """
        # Compute joint positions required to reach the target end-effector position
        joint_poses = p.calculateInverseKinematics(
            self.robot_id, self.end_effector_index, target_position
        )

        # Apply calculated joint positions using POSITION_CONTROL on each joint
        for i in range(len(joint_poses)):
            p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, joint_poses[i])

    def get_end_effector_position(self):
        """
        Retrieve the current position of the end effector.
        
        Returns:
        - list: The (x, y, z) coordinates of the end effector.
        """
        link_state = p.getLinkState(self.robot_id, self.end_effector_index)
        return link_state[0]

    def get_current_joint_positions(self):
        """
        Retrieve current positions of all joints.
        
        Returns:
        - list: Joint positions.
        """
        joint_positions = []
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_state = p.getJointState(self.robot_id, i)
            joint_positions.append(joint_state[0])
        return joint_positions
