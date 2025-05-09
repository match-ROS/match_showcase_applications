import subprocess
import yaml
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtCore import QTimer
import tf.transformations as tf_trans
from rosgraph_msgs.msg import Log

class ROSInterface:
    def __init__(self, gui):
        self.gui = gui
        self.workspace_name = "catkin_ws_recker"
        self.virtual_object_pose = None
        
    def update_poses(self):
        thread = threading.Thread(target=self.subscribe_to_relative_poses, daemon=True)
        thread.start()

    def get_virtual_object_pose_once(self):
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("ros_interface_gui", anonymous=True)
            data = rospy.wait_for_message("/virtual_object/object_pose", PoseStamped, timeout=5)
            position = data.pose.position
            orientation = data.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            euler_angles = tf_trans.euler_from_quaternion(quaternion)
            
            self.virtual_object_pose = [position.x, position.y, euler_angles[2]]
            self.gui.update_virtual_object_pose(self.virtual_object_pose)
            print("Successfully retrieved virtual object pose.", self.virtual_object_pose)
        except rospy.ROSException:
            print("Failed to retrieve virtual object pose within timeout.")

    def move_virtual_object_to_initial_pose(self):
        self.gui.load_relative_poses()

        if self.virtual_object_pose is None or self.virtual_object_pose == [0, 0, 0]:
            print("No virtual object pose available to move.")
            return
        
        x, y, yaw = self.virtual_object_pose
        command = f"roslaunch cooperative_handling move_object_to_initial_pose.launch x:={x} y:={y} yaw:={yaw}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def subscribe_to_relative_poses(self):
        selected_robots = self.gui.get_selected_robots()

        if not selected_robots:
            print("No robots selected. Skipping update.")
            return

        if not rospy.core.is_initialized():
            rospy.init_node("update_relative_poses", anonymous=True, disable_signals=True)
        self.updated_poses = {}

        def callback(data, robot):
            position = data.pose.position
            yaw = tf_trans.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]
            self.updated_poses[robot] = [position.x, position.y, yaw]
            print(f"✅ Received pose for {robot}: {self.updated_poses[robot]}")
            
            if len(self.updated_poses) >= len(selected_robots):
                rospy.signal_shutdown("Pose update complete")
                self.save_poses_to_yaml()

        for robot in selected_robots:
            topic_name = f"/{robot}/relative_pose"
            rospy.Subscriber(topic_name, PoseStamped, callback, robot)
            print(f"Subscribed to {topic_name}")

        rospy.spin()

    def save_poses_to_yaml(self):
        poses = {robot: pose for robot, pose in self.updated_poses.items()}
        self.gui.save_relative_poses(poses)
        self.gui.load_relative_poses()
        self.gui.table.viewport().update()
        self.gui.btn_save_poses.setEnabled(True)

    def start_roscore(self):
        command = "ssh -t -t roscore 'source ~/.bashrc; source /opt/ros/noetic/setup.bash; roscore; exec bash'"
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])

    def start_mocap(self):
        command = "ssh -t -t roscore 'source ~/.bashrc; source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch launch_mocap mocap_launch.launch; exec bash'"
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])

    def launch_drivers(self):
        selected_robots = self.gui.get_selected_robots()
        for robot in selected_robots:
            command = f"ssh -t -t {robot} 'source ~/.bashrc; roslaunch mur_launch_hardware {robot}.launch launch_ur_r:=false launch_ur_l:=false ; exec bash'"
            print(f"Launching driver for {robot}...")
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])

    def update_button_status(self):
        roscore_running = self.is_ros_node_running("/rosout")
        mocap_running = self.is_ros_node_running("/qualisys")
        self.gui.btn_roscore.setStyleSheet("background-color: lightgreen;" if roscore_running else "background-color: lightgray;")
        self.gui.btn_mocap.setStyleSheet("background-color: lightgreen;" if mocap_running else "background-color: lightgray;")

    def is_ros_node_running(self, node_name):
        try:
            output = subprocess.check_output("rosnode list", shell=True).decode()
            return node_name in output.split("\n")
        except subprocess.CalledProcessError:
            return False

    def open_rviz(self):
        command = "roslaunch cooperative_handling launch_rviz.launch"
        subprocess.Popen(command, shell=True)

    def launch_ros(self, package, launch_file):
        selected_robots = self.gui.get_selected_robots()
        robot_names_str = "[" + ",".join(f"'{r}'" for r in selected_robots) + "]"

        command = f"roslaunch {package} {launch_file} robot_names:={robot_names_str}"
        print(f"Executing: {command}")
        subprocess.Popen(command, shell=True)

    def start_sync(self):
        """Starts file synchronization between workspace and selected robots."""
        selected_robots = self.gui.get_selected_robots()
        self.gui.btn_sync.setStyleSheet("background-color: lightgreen;")  # Mark sync as active
        
        for robot in selected_robots:
            command = f"while inotifywait -r -e modify,create,delete,move ~/{self.workspace_name}/src; do \n" \
                      f"rsync --delete -avzhe ssh ~/{self.workspace_name}/src rosmatch@{robot}:~/{self.workspace_name}/ \n" \
                      "done"
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"]) 


    def move_mir_to_start_pose(self):
        """Moves each selected robot to its initial pose via 'roslaunch cooperative_transport move_mir_to_start_pose.launch'.\n        The absolute pose is computed from the Virtual Object pose plus the relative offset from the table."""
        selected_robots = self.gui.get_selected_robots()
        # We need the table values for 'Virtual Object' as the reference
        self.gui.load_relative_poses()  # Ensure table is updated

        # Grab the Virtual Object pose from the table
        vo_row = self.gui.table.rowCount() - 1  # last row is "Virtual Object"
        vo_x = float(self.gui.table.item(vo_row, 0).text()) if self.gui.table.item(vo_row, 0) else 0.0
        vo_y = float(self.gui.table.item(vo_row, 1).text()) if self.gui.table.item(vo_row, 1) else 0.0
        vo_yaw = float(self.gui.table.item(vo_row, 2).text()) if self.gui.table.item(vo_row, 2) else 0.0

        for robot in selected_robots:
            # find the row for this robot in the table
            row_index = None
            for row in range(self.gui.table.rowCount()):
                row_label = self.gui.table.verticalHeaderItem(row).text()
                if row_label == robot:
                    row_index = row
                    break
            if row_index is None:
                print(f"No table entry for {robot}")
                continue

            rel_x = float(self.gui.table.item(row_index, 0).text()) if self.gui.table.item(row_index, 0) else 0.0
            rel_y = float(self.gui.table.item(row_index, 1).text()) if self.gui.table.item(row_index, 1) else 0.0
            rel_yaw = float(self.gui.table.item(row_index, 2).text()) if self.gui.table.item(row_index, 2) else 0.0

            # Compute absolute pose
            abs_x = vo_x + rel_x
            abs_y = vo_y + rel_y
            abs_yaw = vo_yaw + rel_yaw

            target_pose_str = f"[{abs_x},{abs_y},{abs_yaw}]"
            tf_prefix = robot

            command = f"roslaunch cooperative_transport move_mir_to_start_pose.launch tf_prefix:={tf_prefix} target_pose:={target_pose_str}"
            print(f"Moving {robot} to initial pose: {command}")
            #subprocess.Popen(command, shell=True)
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                f"{command}; exec bash"
            ])
