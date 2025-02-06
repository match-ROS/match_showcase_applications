import subprocess
import yaml
import threading

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
    active_counts = {"wrench": 0, "twist": 0, "arm": 0, "admittance": 0}
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
    Wrench: {}/{} {}
    Twist: {}/{} {}
    Arm: {}/{} {}
    Admittance: {}/{} {}
    """.format(
        active_counts["wrench"], total_count, get_status_symbol(active_counts["wrench"], total_count),
        active_counts["twist"], total_count, get_status_symbol(active_counts["twist"], total_count),
        active_counts["arm"], total_count, get_status_symbol(active_counts["arm"], total_count),
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
            command = f"ssh -t -t {robot} 'source ~/.bashrc; export ROS_MASTER_URI=http://roscore:11311/; source /opt/ros/noetic/setup.bash; source ~/{self.workspace_name}/devel/setup.bash; roslaunch manipulator_control dezentralized_admittance_controller.launch tf_prefix:={robot} UR_prefix:={ur_prefix} set_reference_at_runtime:={set_reference}; exec bash'"
            print(f"Executing SSH Command: {command}")

            # Open a new terminal and run the SSH command
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"])