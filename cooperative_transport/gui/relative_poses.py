import yaml
import os, sys

class RelativePoses:
    def __init__(self, filename="poses.yaml"):
        # get script folder
        script_dir = os.path.dirname(__file__)
        abs_file_path = os.path.join(script_dir, filename)
        self.filename = abs_file_path
        
        #self.filename = filename

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
