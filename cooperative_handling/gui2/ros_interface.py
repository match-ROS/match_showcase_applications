import subprocess
import yaml
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QTableWidgetItem

import rospy
from geometry_msgs.msg import PoseStamped

class ROSInterface:
    def __init__(self, gui):
        self.gui = gui
        self.workspace_name = "catkin_ws_recker"
        self.updated_poses = {}  # Zwischenspeicher für empfangene Posen

    def update_poses(self):
        """Startet einen Thread für das Abonnieren der relativen Posen."""
        thread = threading.Thread(target=self.subscribe_to_relative_poses, daemon=True)
        thread.start()


    def subscribe_to_relative_poses(self):
        """Abonniert die relativen Posen der ausgewählten Roboter."""
        selected_robots = self.gui.get_selected_robots()
        selected_urs = self.gui.get_selected_urs()

        if not selected_robots or not selected_urs:
            print("No robots or URs selected. Skipping update.")
            return

        rospy.init_node("update_relative_poses", anonymous=True, disable_signals=True)  # Verhindert Blockieren

        def callback(data, robot_ur):
            """Speichert die empfangenen Positionen und startet GUI-Update im Haupt-Thread."""
            self.updated_poses[robot_ur] = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
            print(f"Received pose for {robot_ur}: {self.updated_poses[robot_ur]}")

            # Falls alle selektierten Roboter Daten empfangen haben, update Tabelle über den Qt-Hauptthread
            if len(self.updated_poses) >= len(selected_robots) * len(selected_urs):
                rospy.signal_shutdown("Pose update complete")
                self.gui.invokeMethod(self.gui, "insert_updated_poses")


        for robot in selected_robots:
            for ur in selected_urs:
                topic_name = f"/{robot}/{ur}/relative_pose"
                rospy.Subscriber(topic_name, PoseStamped, callback, (robot, ur))
                print(f"Subscribed to {topic_name}")

        rospy.spin()  # Hält den Subscriber aktiv, blockiert aber nicht die GUI

    def insert_updated_poses(self):
        """Trägt die aktualisierten Werte in die Tabelle ein und aktiviert den Save-Button."""
        for row in range(self.gui.table.rowCount()):
            robot_ur = self.gui.table.verticalHeaderItem(row).text()
            if robot_ur in self.updated_poses:
                x, y, z = self.updated_poses[robot_ur]

                # GUI-Elemente in den Haupt-Thread übergeben
                self.gui.table.setItem(row, 0, QTableWidgetItem(f"{x:.4f}"))
                self.gui.table.setItem(row, 1, QTableWidgetItem(f"{y:.4f}"))
                self.gui.table.setItem(row, 2, QTableWidgetItem(f"{z:.4f}"))

        # Aktiviert den "Save Poses"-Button nach dem ersten Update
        self.gui.btn_save_poses.setEnabled(True)
        print("Updated table with received poses.")

    def insert_updated_poses(self):
        """Trägt die aktualisierten Werte in die Tabelle ein und aktiviert den Save-Button."""
        for row in range(self.gui.table.rowCount()):
            robot_ur = self.gui.table.verticalHeaderItem(row).text()
            if robot_ur in self.updated_poses:
                x, y, z = self.updated_poses[robot_ur]
                self.gui.table.setItem(row, 0, QTableWidgetItem(str(x)))
                self.gui.table.setItem(row, 1, QTableWidgetItem(str(y)))
                self.gui.table.setItem(row, 2, QTableWidgetItem(str(z)))

        # Aktiviert den "Save Poses"-Button nach dem ersten Update
        self.gui.btn_save_poses.setEnabled(True)




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
    """SSH into each selected robot and start the cooperative admittance controller."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    set_reference = "true" if gui.check_set_reference.isChecked() else "false"

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    for robot in selected_robots:
        for ur_prefix in selected_urs:
            command = f"ssh -t -t {robot} 'source ~/.bashrc; export ROS_MASTER_URI=http://roscore:11311/; source /opt/ros/noetic/setup.bash; source ~/{gui.workspace_name}/devel/setup.bash; roslaunch manipulator_control dezentralized_admittance_controller.launch tf_prefix:={robot} UR_prefix:={ur_prefix} set_reference_at_runtime:={set_reference}; exec bash'"
            print(f"Executing SSH Command: {command}")

            # Open a new terminal and run the SSH command
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])