import csv
import os

class Logger:
    def __init__(self, config):
        """
        Initializes the Logger object to log both robot and rope states.
        
        Parameters:
        - config: dictionary from the configuration file containing paths and settings.
        """
        # Get log file paths from config
        self.robot_log_path = config['robot_log_path']
        self.rope_log_path = config['rope_log_path']

        # Create directories if they do not exist
        os.makedirs(os.path.dirname(self.robot_log_path), exist_ok=True)
        os.makedirs(os.path.dirname(self.rope_log_path), exist_ok=True)

        # Open both files for writing
        self.robot_log_file = open(self.robot_log_path, mode='w', newline='')
        self.rope_log_file = open(self.rope_log_path, mode='w', newline='')

        # Create CSV writers
        self.robot_writer = csv.writer(self.robot_log_file)
        self.rope_writer = csv.writer(self.rope_log_file)

        # Write headers to the CSV files
        self._write_headers()

    def _write_headers(self):
        # Headers for robot states log
        robot_headers = [
            'time', 'end_effector_x', 'end_effector_y', 'end_effector_z',
            'end_effector_qx', 'end_effector_qy', 'end_effector_qz', 'end_effector_qw',
            'end_effector_vx', 'end_effector_vy', 'end_effector_vz',
            'end_effector_angular_vx', 'end_effector_angular_vy', 'end_effector_angular_vz',
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'
        ]
        self.robot_writer.writerow(robot_headers)

        # Headers for rope states log
        rope_headers = ['time']
        for i in range(self.config['rope']['num_segments']):
            rope_headers += [
                f'segment_{i}_x', f'segment_{i}_y', f'segment_{i}_z',
                f'segment_{i}_vx', f'segment_{i}_vy', f'segment_{i}_vz'
            ]
        self.rope_writer.writerow(rope_headers)

    def log_robot_state(self, time, robot_control):
        """
        Logs the state of the robot.
        
        Parameters:
        - time: The current time step of the simulation.
        - robot_control: Instance of RobotControl to get the current state of the robot.
        """
        # Get end-effector state
        link_state = robot_control.get_end_effector_state()
        end_effector_pos = [round(x, 4) for x in link_state['position']]
        end_effector_ori = [round(x, 4) for x in link_state['orientation']]
        end_effector_lin_vel = [round(x, 4) for x in link_state['linear_velocity']]
        end_effector_ang_vel = [round(x, 4) for x in link_state['angular_velocity']]

        # Collect joint angles
        joint_positions = [round(x, 4) for x in robot_control.get_joint_positions()]

        # Write data to CSV
        self.robot_writer.writerow(
            [round(time, 4), *end_effector_pos, *end_effector_ori,
             *end_effector_lin_vel, *end_effector_ang_vel, *joint_positions]
        )

    def log_rope_state(self, time, rope_setup):
        """
        Logs the state of the rope.
        
        Parameters:
        - time: The current time step of the simulation.
        - rope_setup: Instance of RopeSetup to get the current state of the rope.
        """
        rope_data = [round(time, 4)]
        for segment_id in rope_setup.rope_segments:
            seg_pos, _ = p.getBasePositionAndOrientation(segment_id)
            seg_lin_vel, _ = p.getBaseVelocity(segment_id)
            rope_data.extend([round(x, 4) for x in seg_pos])
            rope_data.extend([round(x, 4) for x in seg_lin_vel])

        # Write data to CSV
        self.rope_writer.writerow(rope_data)

    def close_logs(self):
        """
        Closes the CSV files when logging is complete.
        """
        self.robot_log_file.close()
        self.rope_log_file.close()

