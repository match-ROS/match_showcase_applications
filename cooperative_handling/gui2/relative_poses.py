import yaml

class RelativePoses:
    def __init__(self, filename="poses.yaml"):
        self.filename = filename

    def load_poses(self):
        """Loads relative poses from a YAML file."""
        try:
            with open(self.filename, "r") as file:
                return yaml.safe_load(file)
        except Exception as e:
            print(f"Error loading poses: {e}")
            return {}

    def save_poses(self, poses):
        """Saves relative poses to a YAML file."""
        try:
            with open(self.filename, "w") as file:
                yaml.safe_dump(poses, file)
        except Exception as e:
            print(f"Error saving poses: {e}")
