class TrajectoryBase:
    def __init__(self, params):
        self.params = params

    def get_target_position(self, t):
        raise NotImplementedError("This method should be implemented by subclasses.")
