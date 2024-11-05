import csv
import os
import pybullet as p
from datetime import datetime

class Logger:
    def __init__(self, config, log_dir=None):
        self.config = config
        
        # Determine the log directory or use default from config
        if log_dir:
            self.robot_log_path = os.path.join(log_dir, "robot_states_log.csv")
            self.rope_log_path = os.path.join(log_dir, "rope_states_log.csv")
        else:
            self.robot_log_path = config['logging']['robot_log_path']
            self.rope_log_path = config['logging']['rope_log_path']
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(self.robot_log_path), exist_ok=True)
        os.makedirs(os.path.dirname(self.rope_log_path), exist_ok=True)

        # Open files for logging
        self.robot_log_file = open(self.robot_log_path, mode='w', newline='')
        self.rope_log_file = open(self.rope_log_path, mode='w', newline='')

        self.robot_writer = csv.writer(self.robot_log_file)
        self.rope_writer = csv.writer(self.rope_log_file)

        # Write headers for logs
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
            rope_headers += [f'segment_{i}_x', f'segment_{i}_y', f'segment_{i}_z',
                             f'segment_{i}_vx', f'segment_{i}_vy', f'segment_{i}_vz']
        self.rope_writer.writerow(rope_headers)

    def log(self, time_step, robot_control, rope_setup):
        # Get robot state data
        link_state = p.getLinkState(robot_control.robot_id, robot_control.end_effector_index, computeLinkVelocity=True)
        end_effector_pos = [round(x, 4) for x in link_state[0]]
        end_effector_ori = [round(x, 4) for x in link_state[1]]
        end_effector_lin_vel = [round(x, 4) for x in link_state[6]]
        end_effector_ang_vel = [round(x, 4) for x in link_state[7]]
        
        # Collect joint data
        joint_data = [round(p.getJointState(robot_control.robot_id, i)[0], 4) for i in range(7)]

        # Write robot states to CSV
        self.robot_writer.writerow([round(time_step, 4), *end_effector_pos, *end_effector_ori, *end_effector_lin_vel, *end_effector_ang_vel, *joint_data])

        # Collect rope segment states
        rope_data = [round(time_step, 4)]
        for segment_id in rope_setup.rope_segments:
            seg_pos, _ = p.getBasePositionAndOrientation(segment_id)
            seg_lin_vel, _ = p.getBaseVelocity(segment_id)
            rope_data.extend([round(x, 4) for x in seg_pos])
            rope_data.extend([round(x, 4) for x in seg_lin_vel])

        # Write rope states to CSV
        self.rope_writer.writerow(rope_data)

    def __del__(self):
        # Ensure to close log files
        self.robot_log_file.close()
        self.rope_log_file.close()
