import pybullet as p
import pybullet_data

class RobotControl:
    def __init__(self, robot_params):
        # Set additional search path for PyBullet to load URDF files properly
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load the Franka Panda robot URDF
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
        self.end_effector_index = 11

        # Initialize joints to the provided initial positions
        self.initial_joint_positions = robot_params["initial_joint_positions"]
        for i, pos in enumerate(self.initial_joint_positions):
            p.resetJointState(self.robot_id, i, pos)

    def move_end_effector(self, target_position):
        """
        Move the end effector to a given target position using inverse kinematics.
        
        Args:
        - target_position (list): The target (x, y, z) coordinates for the end effector.
        """
        # Calculate the inverse kinematics to get the joint angles needed to reach the target position
        joint_poses = p.calculateInverseKinematics(
            self.robot_id, self.end_effector_index, target_position
        )
        
        # Set each joint's position directly
        for i in range(len(joint_poses)):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[i]
            )

        # Step the simulation for smooth transition
        p.stepSimulation()

        # Print actual end-effector position for verification
        actual_position = self.get_end_effector_position()
        #print(f"Target Position: {target_position}, Actual Position: {actual_position}")

    def get_end_effector_position(self):
        """
        Get the current position of the end effector.

        Returns:
        - list: The (x, y, z) coordinates of the end effector.
        """
        link_state = p.getLinkState(self.robot_id, self.end_effector_index)
        return link_state[0]  # Returns the position (x, y, z)

    def get_current_joint_positions(self):
        """
        Get the current positions of all the joints in the robot.

        Returns:
        - list: A list containing the positions of all joints.
        """
        joint_positions = []
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_state = p.getJointState(self.robot_id, i)
            joint_positions.append(joint_state[0])  # Get the joint position
        return joint_positions
