import pybullet as p


class RopeSetup:
    def __init__(self, rope_params, robot_id, end_effector_index):
        # Store rope parameters and robot identifiers
        self.rope_params = rope_params
        self.segment_length = rope_params["length"] / rope_params["num_segments"]
        self.num_segments = rope_params["num_segments"]
        self.segment_radius = rope_params["segment_radius"]
        self.segment_mass = rope_params["segment_mass"]
        self.rope_segments = []
        self.robot_id = robot_id
        self.end_effector_index = end_effector_index
        self.ghost_anchor_id = None
        self.ghost_anchor_constraint = None

    def create_rope(self, end_effector_position):
        """
        Create the rope segments hanging below the robot's end-effector.
        """
        # Create segments directly below the given start position
        for i in range(self.num_segments):
            segment_pos = [
                end_effector_position[0],
                end_effector_position[1],
                end_effector_position[2] - (i + 1) * self.segment_length,
            ]

            # Create capsule segment for collision and visualization
            segment_col_shape = p.createCollisionShape(
                shapeType=p.GEOM_CAPSULE,
                radius=self.segment_radius,
                height=self.segment_length,
            )
            segment_vis_shape = p.createVisualShape(
                shapeType=p.GEOM_CAPSULE,
                radius=self.segment_radius,
                length=self.segment_length,
                rgbaColor=[1, 0, 0, 1],
            )
            segment_id = p.createMultiBody(
                baseMass=self.segment_mass,
                baseCollisionShapeIndex=segment_col_shape,
                baseVisualShapeIndex=segment_vis_shape,
                basePosition=segment_pos,
            )

            # Disable collisions between adjacent segments to improve stability
            if i > 0:
                p.setCollisionFilterPair(
                    segment_id, self.rope_segments[i - 1], -1, -1, enableCollision=0
                )

            self.rope_segments.append(segment_id)

        # Step 2: Attach the first rope segment to the robot end-effector AFTER it is in position
        self._attach_to_end_effector()

    def _attach_to_end_effector(self):
        """
        Attach the first rope segment to the robot's end-effector.
        """
        # Use a fixed joint to attach the first segment to the end-effector
        p.createConstraint(
            parentBodyUniqueId=self.robot_id,  # Robot body ID
            parentLinkIndex=self.end_effector_index,  # End-effector link index
            childBodyUniqueId=self.rope_segments[0],  # First rope segment
            childLinkIndex=-1,  # Rope is a free body without links
            jointType=p.JOINT_FIXED,  # Fixed joint to ensure no relative motion
            jointAxis=[0, 0, 0],  # No specific joint axis for fixed joint
            parentFramePosition=[0, 0, 0],  # Attach at the end-effector's center
            childFramePosition=[
                0,
                0,
                -self.segment_length / 2,
            ],  # Attach at the bottom of the first rope segment
        )

    def add_segment_constraints(self):
        """
        Add point-to-point constraints between each consecutive rope segment.
        """
        for i in range(self.num_segments - 1):
            p.createConstraint(
                parentBodyUniqueId=self.rope_segments[i],
                parentLinkIndex=-1,
                childBodyUniqueId=self.rope_segments[i + 1],
                childLinkIndex=-1,
                jointType=p.JOINT_POINT2POINT,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, self.segment_length / 2],
                childFramePosition=[0, 0, -self.segment_length / 2],
            )

    def add_ghost_anchor(self, end_effector_position):
        """
        Create a ghost anchor below the loose end of the rope to help stabilize it during initialization.
        """
        ghost_anchor_position = [
            end_effector_position[0],
            end_effector_position[1],
            end_effector_position[2] - self.segment_length * (self.num_segments + 1),
        ]

        # Create a massless body to act as the ghost anchor
        self.ghost_anchor_id = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1, basePosition=ghost_anchor_position
        )

        # Attach the last rope segment to the ghost anchor point using a fixed constraint
        self.ghost_anchor_constraint = p.createConstraint(
            parentBodyUniqueId=self.rope_segments[-1],
            parentLinkIndex=-1,
            childBodyUniqueId=self.ghost_anchor_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, -self.segment_length / 2],
            childFramePosition=[0, 0, 0],
        )

    def remove_ghost_anchor(self):
        """
        Remove the ghost anchor to allow the rope to hang freely.
        """
        if self.ghost_anchor_constraint is not None:
            p.removeConstraint(self.ghost_anchor_constraint)
