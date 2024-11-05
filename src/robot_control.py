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
        joint_poses = p.calculateInverseKinematics(
            self.robot_id, self.end_effector_index, target_position
        )
        for i in range(len(joint_poses)):
            p.setJointMotorControl2(
                self.robot_id, i, p.POSITION_CONTROL, joint_poses[i]
            )

    def get_end_effector_position(self):
        return p.getLinkState(self.robot_id, self.end_effector_index)[
            0
        ]  # Returns the position (x, y, z)
