import subprocess
import yaml
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtCore import QTimer
import tf.transformations as tf_trans

import rospy
from geometry_msgs.msg import PoseStamped

class ROSInterface:
    def __init__(self, gui):
        self.gui = gui
        self.workspace_name = "catkin_ws_recker"
        self.updated_poses = {}

    def update_poses(self):
        """Startet einen Thread für das Abonnieren der relativen Posen und Speichert in YAML."""
        thread = threading.Thread(target=self.subscribe_to_relative_poses, daemon=True)
        thread.start()

    def subscribe_to_relative_poses(self):
        """Abonniert die relativen Posen der ausgewählten Roboter und speichert sie in YAML."""
        selected_robots = self.gui.get_selected_robots()
        selected_urs = self.gui.get_selected_urs()

        if not selected_robots or not selected_urs:
            print("No robots or URs selected. Skipping update.")
            return

        rospy.init_node("update_relative_poses", anonymous=True, disable_signals=True)
        self.updated_poses = {}

        def callback(data, robot_ur):
            """Receives relative pose and extracts both position (x, y, z) and orientation (Rx, Ry, Rz) in Euler angles."""
            
            # Extract position
            position = data.pose.position
            x, y, z = position.x, position.y, position.z

            # Extract orientation as quaternion
            orientation = data.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            rx, ry, rz = tf_trans.euler_from_quaternion(quaternion)

            # Store the values in the dictionary
            self.updated_poses[robot_ur] = {"x": x, "y": y, "z": z, "rx": rx, "ry": ry, "rz": rz}

            print(f"✅ Received pose for {robot_ur}: {self.updated_poses[robot_ur]}")

            # Check if all poses have been received, then trigger save
            if len(self.updated_poses) >= len(selected_robots) * len(selected_urs):
                rospy.signal_shutdown("Pose update complete")
                self.save_poses_to_yaml()



        for robot in selected_robots:
            for ur in selected_urs:
                topic_name = f"/{robot}/{ur}/relative_pose"
                rospy.Subscriber(topic_name, PoseStamped, callback, (robot, ur))
                print(f"Subscribed to {topic_name}")

        rospy.spin()

    def save_poses_to_yaml(self):
        """Collects received relative poses and saves them in the same format as the table."""
        poses = {}

        # Convert received poses into table-compatible format
        for robot_ur, pose in self.updated_poses.items():
            poses[robot_ur] = [pose["x"], pose["y"], pose["z"], pose["rx"], pose["ry"], pose["rz"]]


        # Save collected poses using the existing method in gui_layout
        self.gui.save_relative_poses(poses)

        # Reload table values immediately
        self.gui.load_relative_poses()
        self.gui.table.viewport().update()  # Forces an immediate redraw

        # Enable the "Save Poses" button after the first update
        self.gui.btn_save_poses.setEnabled(True)

    def start_roscore(self):
        """Starts roscore on the roscore PC."""
        command = "ssh -t -t roscore 'source ~/.bashrc; source /opt/ros/noetic/setup.bash; roscore; exec bash'"
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])

    def start_mocap(self):
        """Starts the motion capture system on the roscore PC."""
        command = "ssh -t -t roscore 'source ~/.bashrc; source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch launch_mocap mocap_launch.launch; exec bash'"
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])

    def start_sync(self):
        """Starts file synchronization between workspace and selected robots."""
        selected_robots = self.gui.get_selected_robots()
        
        for robot in selected_robots:
            command = f"while inotifywait -r -e modify,create,delete,move ~/{self.workspace_name}/src; do \n" \
                      f"rsync --delete -avzhe ssh ~/{self.workspace_name}/src rosmatch@{robot}:~/{self.workspace_name}/ \n" \
                      "done"
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"]) 

    def update_button_status(self):
        """Checks if roscore and mocap are running and updates button colors."""
        roscore_running = self.is_ros_node_running("/rosout")
        mocap_running = self.is_ros_node_running("/qualisys")

        self.gui.btn_roscore.setStyleSheet("background-color: lightgreen;" if roscore_running else "background-color: lightgray;")
        self.gui.btn_mocap.setStyleSheet("background-color: lightgreen;" if mocap_running else "background-color: lightgray;")

    def is_ros_node_running(self, node_name):
        """Checks if a specific ROS node is running by using `rosnode list`."""
        try:
            output = subprocess.check_output("rosnode list", shell=True).decode()
            return node_name in output.split("\n")
        except subprocess.CalledProcessError:
            return False




def launch_ros(gui, package, launch_file):
    selected_robots = gui.get_selected_robots()
    robot_names_str = "[" + ",".join(f"'{r}'" for r in selected_robots) + "]"

    command = f"roslaunch {package} {launch_file} robot_names:={robot_names_str}"
    print(f"Executing: {command}")
    subprocess.Popen(command, shell=True)

def run_compute_object_center(gui):
    selected_robots = gui.get_selected_robots()

    if not selected_robots:
        print("No robots selected. Skipping launch.")
        return

    robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
    command = f"roslaunch cooperative_handling compute_object_center.launch robot_names:={robot_names_str}"
    print(f"Executing: {command}")
    subprocess.Popen(command, shell=True)

def zero_ft_sensors(gui):
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
    ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'
    command = f"roslaunch cooperative_handling zero_all_FT_sensors.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
    print(f"Executing: {command}")
    subprocess.Popen(command, shell=True)

def start_status_update(gui):
    threading.Thread(target=update_status, args=(gui,), daemon=True).start()

def update_status(gui):
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    active_counts = {"force_torque_sensor_controller": 0, "twist_controller": 0, "arm_controller": 0, "admittance": 0}
    total_count = len(selected_robots) * len(selected_urs)
    
    for robot in selected_robots:
        for ur in selected_urs:
            service_name = f"/{robot}/{ur}/controller_manager/list_controllers"
            try:
                output = subprocess.check_output(f"rosservice call {service_name}", shell=True).decode()
                controllers = yaml.safe_load(output).get("controller", [])
                for controller in controllers:
                    if controller.get("state") == "running":
                        active_counts[controller["name"]] += 1
            except Exception:
                pass
    
    status_text = """
    Force/Torque Sensor: {}/{} {}
    Twist Controller: {}/{} {}
    Arm Controller: {}/{} {}
    Admittance Controller: {}/{} {}
    """.format(
        active_counts["force_torque_sensor_controller"], total_count, get_status_symbol(active_counts["force_torque_sensor_controller"], total_count),
        active_counts["twist_controller"], total_count, get_status_symbol(active_counts["twist_controller"], total_count),
        active_counts["arm_controller"], total_count, get_status_symbol(active_counts["arm_controller"], total_count),
        active_counts["admittance"], total_count, get_status_symbol(active_counts["admittance"], total_count),
    )
    
    gui.status_label.setText(status_text)

def get_status_symbol(active, total):
    if active == total:
        return "✅"
    elif active > 0:
        return "⚠️"
    return "❌"

def open_rviz():
    command = "roslaunch cooperative_handling launch_rviz.launch"
    subprocess.Popen(command, shell=True)

def launch_drivers(gui):
    selected_robots = gui.get_selected_robots()
    for robot in selected_robots:
        command = f"ssh -t -t {robot} 'source ~/.bashrc; roslaunch mur_launch_hardware {robot}.launch; exec bash'"
        print(f"Starting driver for {robot}...")
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])

def quit_drivers():
    print("Stopping all drivers...")
    subprocess.Popen("pkill -f 'roslaunch'", shell=True)

def move_to_initial_pose(gui, UR_prefix):
    selected_robots = gui.get_selected_robots()
    move_group_name = "UR_arm_l" if UR_prefix == "UR10_l" else "UR_arm_r"
    for robot in selected_robots:
        home_position = "handling_position_wide" if robot in ["mur620a", "mur620b"] else "handling_position_wide_lift"
        command = f"ROS_NAMESPACE={robot} roslaunch ur_utilities move_UR_to_home_pose.launch tf_prefix:={robot} UR_prefix:={UR_prefix} home_position:={home_position} move_group_name:={move_group_name}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)


def turn_on_wrench_controllers(gui):
    """Turns on all wrench controllers for the selected robots."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
    ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

    command = f"roslaunch cooperative_handling turn_on_all_wrench_controllers.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
    print(f"Executing: {command}")
    subprocess.Popen(command, shell=True)

def turn_on_arm_controllers(gui):
    """Turns on all arm controllers for the selected robots."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
    ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

    command = f"roslaunch cooperative_handling turn_on_all_arm_controllers.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
    print(f"Executing: {command}")
    subprocess.Popen(command, shell=True)

def turn_on_twist_controllers(gui):
    """Turns on all twist controllers for the selected robots."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
    ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

    command = f"roslaunch cooperative_handling turn_on_all_twist_controllers.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
    print(f"Executing: {command}")
    subprocess.Popen(command, shell=True)

def enable_all_urs(gui):
    """Enables all UR robots for the selected configurations."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
    ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

    command = f"roslaunch cooperative_handling enable_all_URs.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
    print(f"Executing: {command}")
    subprocess.Popen(command, shell=True)

def update_ur_relative_to_object(gui):
    """Updates the relative poses of UR robots to the object."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    robot_names_str = '"' + str(selected_robots).replace("'", '"') + '"'
    ur_prefixes_str = '"' + str(selected_urs).replace("'", '"') + '"'

    command = f"roslaunch cooperative_handling update_all_relative_poses.launch robot_names:={robot_names_str} UR_prefixes:={ur_prefixes_str}"
    print(f"Executing: {command}")
    subprocess.Popen(command, shell=True)


def launch_drivers(gui):
    """SSH into the selected robots and start the drivers in separate terminals."""
    selected_robots = gui.get_selected_robots()

    for robot in selected_robots:
        workspace = gui.workspace_name
        command = f"ssh -t -t {robot} 'source ~/.bashrc; export ROS_MASTER_URI=http://roscore:11311/; source /opt/ros/noetic/setup.bash; source ~/{workspace}/devel/setup.bash; roslaunch mur_launch_hardware {robot}.launch; exec bash'"
        print(f"Opening SSH session and launching driver for: {robot}")

        # Open a new terminal with SSH session + driver launch + keep open
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])

def quit_drivers(gui):
    """Terminates all running driver sessions and closes terminals."""
    print("Stopping all driver sessions...")
    try:
        subprocess.Popen("pkill -f 'ssh -t -t'", shell=True)
        subprocess.Popen("pkill -f 'gnome-terminal'", shell=True)
    except Exception as e:
        print(f"Error stopping processes: {e}")


def move_to_initial_pose(gui, UR_prefix):
    """Moves the selected robots to the initial pose with the correct namespace and move_group_name."""
    selected_robots = gui.get_selected_robots()

    # Set move_group_name based on UR_prefix
    move_group_name = "UR_arm_l" if UR_prefix == "UR10_l" else "UR_arm_r"

    for robot in selected_robots:
        # Set home_position based on robot name
        if robot in ["mur620a", "mur620b"]:
            home_position = "handling_position_wide"
        else:  # mur620c, mur620d
            home_position = "handling_position_wide_lift"

        # ROS launch command with namespace
        command = f"ROS_NAMESPACE={robot} roslaunch ur_utilities move_UR_to_home_pose.launch tf_prefix:={robot} UR_prefix:={UR_prefix} home_position:={home_position} move_group_name:={move_group_name}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

def turn_on_coop_admittance_controller(gui):
    """Starts the decentralized admittance controller with the correct relative pose for each selected robot."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    set_reference = "true" if gui.check_set_reference.isChecked() else "false"

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    for robot in selected_robots:
        for ur_prefix in selected_urs:
            # Get the relative pose from the table
            relative_pose = gui.get_relative_pose(robot, ur_prefix)

            # Convert list to ROS parameter format
            relative_pose_str = f"[{relative_pose[0]},{relative_pose[1]},{relative_pose[2]},{relative_pose[3]},{relative_pose[4]},{relative_pose[5]}]"

            command = f"ssh -t -t {robot} 'source ~/.bashrc; export ROS_MASTER_URI=http://roscore:11311/; source /opt/ros/noetic/setup.bash; source ~/{gui.workspace_name}/devel/setup.bash; roslaunch manipulator_control dezentralized_admittance_controller.launch tf_prefix:={robot} UR_prefix:={ur_prefix} set_reference_at_runtime:={set_reference} relative_pose:={relative_pose_str}; exec bash'"
            
            print(f"Executing SSH Command: {command}")

            # Open a new terminal and run the SSH command
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])
