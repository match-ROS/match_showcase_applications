import threading
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTableWidget, QTableWidgetItem, QCheckBox
from PyQt5.QtCore import QTimer, Qt
from ros_interface import start_status_update, open_rviz, run_compute_object_center, launch_drivers, quit_drivers, zero_ft_sensors, turn_on_wrench_controllers, turn_on_arm_controllers, turn_on_twist_controllers, enable_all_urs, update_ur_relative_to_object, launch_ros, move_to_initial_pose, turn_on_coop_admittance_controller
from relative_poses import load_relative_poses, save_relative_poses

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Robot Demo")
        self.setGeometry(100, 100, 800, 500)
        
        main_layout = QHBoxLayout()
        
        # Left Side (Status & Buttons)
        left_layout = QVBoxLayout()
        self.status_label = QLabel("Controller Status: Not Checked")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("border: 1px solid black; padding: 5px;")
        left_layout.addWidget(self.status_label)

        # Layout for robot checkboxes and UR checkboxes side by side
        robot_ur_layout = QHBoxLayout()

        # Left column: Robot checkboxes
        robot_layout = QVBoxLayout()
        self.robots = {
            "mur620a": QCheckBox("mur620a"),
            "mur620b": QCheckBox("mur620b"),
            "mur620c": QCheckBox("mur620c"),
            "mur620d": QCheckBox("mur620d"),
        }
        for checkbox in self.robots.values():
            robot_layout.addWidget(checkbox)

        # Right column: UR checkboxes
        ur_layout = QVBoxLayout()
        ur_layout.addWidget(QLabel("Select URs:"))  # Section label
        self.ur10_l = QCheckBox("UR10_l")
        self.ur10_r = QCheckBox("UR10_r")

        # Set UR checkboxes to be selected by default
        self.ur10_l.setChecked(True)
        self.ur10_r.setChecked(True)

        ur_layout.addWidget(self.ur10_l)
        ur_layout.addWidget(self.ur10_r)

        # Add both layouts to the horizontal layout
        robot_ur_layout.addLayout(robot_layout)
        robot_ur_layout.addLayout(ur_layout)
        
        # Timer for status updates
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(lambda: start_status_update(self))
        self.status_timer.start(30000)
        
        # Buttons for ROS Actions
        buttons = {
            "Check Status": lambda: start_status_update(self),
            "Open RVIZ": open_rviz,
            "Start Virtual Leader": virtual_leader,
            "Start Virtual Object": start_virtual_object,
            "Compute Object Center": start_compute_object_center,
            "Zero F/T Sensors": zero_ft_sensors,
            "Turn on Wrench Controllers": turn_on_wrench_controllers,
            "Turn on Arm Controllers": turn_on_arm_controllers,
            "Turn on Twist Controllers": turn_on_twist_controllers,
            "Enable all URs": enable_all_URs,
            "Update UR relative to object": update_UR_relative_to_object,
            "Move to Initial Pose": move_to_initial_pose,
            "Turn on Admittance Controller": turn_on_admittance_controller
        }

        for text, function in buttons.items():
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, f=function: f(self))
            left_layout.addWidget(btn)
        
        # Buttons for ROS actions
        self.btn_virtual_leader = QPushButton("Start Virtual Leader")
        self.btn_virtual_leader.clicked.connect(lambda: launch_ros("virtual_leader", "virtual_leader.launch"))

        self.btn_virtual_object = QPushButton("Start Virtual Object")
        self.btn_virtual_object.clicked.connect(lambda: launch_ros("virtual_object", "virtual_object.launch"))

        self.btn_compute_center = QPushButton("Start Compute Object Center")
        self.btn_compute_center.clicked.connect(run_compute_object_center)

        self.btn_launch_drivers = QPushButton("Launch Drivers")
        self.btn_launch_drivers.clicked.connect(launch_drivers)

        self.btn_quit_drivers = QPushButton("Quit Drivers")
        self.btn_quit_drivers.clicked.connect(quit_drivers)

        self.btn_check_status = QPushButton("Check Status")
        self.btn_check_status.clicked.connect(lambda: start_status_update(self))
        left_layout.addWidget(self.btn_check_status)
        
        self.btn_open_rviz = QPushButton("Open RVIZ")
        self.btn_open_rviz.clicked.connect(open_rviz)

        self.btn_zero_ft_sensors = QPushButton("Zero F/T Sensors")
        self.btn_zero_ft_sensors.clicked.connect(zero_ft_sensors)

        self.btn_turn_on_wrench = QPushButton("Turn on Wrench Controllers")
        self.btn_turn_on_wrench.clicked.connect(turn_on_wrench_controllers)

        self.btn_turn_on_arm = QPushButton("Turn on Arm Controllers")
        self.btn_turn_on_arm.clicked.connect(turn_on_arm_controllers)

        self.btn_turn_on_twist = QPushButton("Turn on Twist Controllers")
        self.btn_turn_on_twist.clicked.connect(turn_on_twist_controllers)

        self.btn_enable_all_urs = QPushButton("Enable all URs")
        self.btn_enable_all_urs.clicked.connect(enable_all_urs)

        self.btn_update_ur_relative = QPushButton("Update UR relative to object")
        self.btn_update_ur_relative.clicked.connect(update_ur_relative_to_object)

        # Buttons for Initial Pose (left & right)
        move_pose_layout = QHBoxLayout()
        self.btn_move_left = QPushButton("Move to Initial Pose Left")
        self.btn_move_left.clicked.connect(lambda: move_to_initial_pose("UR10_l"))
        self.btn_move_right = QPushButton("Move to Initial Pose Right")
        self.btn_move_right.clicked.connect(lambda: move_to_initial_pose("UR10_r"))

        # Layout for the cooperative admittance controller button and its checkbox
        self.btn_coop_admittance = QPushButton("Turn on cooperative Admittance Controller")
        self.btn_coop_admittance.setStyleSheet("background-color: orange; color: black; font-weight: bold;")  # Highlight button
        self.btn_coop_admittance.clicked.connect(turn_on_coop_admittance_controller)
        self.check_set_reference = QCheckBox("Set reference at runtime")
        self.check_set_reference.setChecked(True)  # Pre-select checkbox
        admittance_layout = QHBoxLayout()
        admittance_layout.addWidget(self.btn_coop_admittance)
        admittance_layout.addWidget(self.check_set_reference)



        move_pose_layout.addWidget(self.btn_move_left)
        move_pose_layout.addWidget(self.btn_move_right)
        # Add elements to the left layout
        left_layout.addWidget(self.btn_open_rviz)
        left_layout.addLayout(robot_ur_layout)  # Robot & UR checkboxes
        left_layout.addWidget(self.btn_virtual_leader)
        left_layout.addWidget(self.btn_virtual_object)
        left_layout.addWidget(self.btn_compute_center)
        left_layout.addWidget(self.btn_launch_drivers)
        left_layout.addWidget(self.btn_quit_drivers)
        left_layout.addWidget(self.btn_open_rviz)
        left_layout.addWidget(self.btn_zero_ft_sensors)
        left_layout.addLayout(move_pose_layout)  # Move buttons
        left_layout.addWidget(self.btn_turn_on_wrench)
        left_layout.addWidget(self.btn_turn_on_arm)
        left_layout.addWidget(self.btn_turn_on_twist)
        left_layout.addWidget(self.btn_enable_all_urs)
        left_layout.addWidget(self.btn_update_ur_relative)
        left_layout.addLayout(admittance_layout)  # Admittance controller

        main_layout.addLayout(left_layout)
        
        # Right Side (Table for Relative Poses)
        self.table = QTableWidget(8, 3)
        self.table.setHorizontalHeaderLabels(["X", "Y", "Z"])
        self.table.setVerticalHeaderLabels([
            "mur620a/UR10_l", "mur620a/UR10_r", "mur620b/UR10_l", "mur620b/UR10_r", 
            "mur620c/UR10_l", "mur620c/UR10_r", "mur620d/UR10_l", "mur620d/UR10_r"
        ])
        load_relative_poses(self.table)
        main_layout.addWidget(self.table)
        
        # Save Button
        self.btn_save_poses = QPushButton("Save Poses")
        self.btn_save_poses.clicked.connect(lambda: save_relative_poses(self.table))
        main_layout.addWidget(self.btn_save_poses)
        
        self.setLayout(main_layout)


    def get_selected_robots(self):
            """Returns a list of selected robots."""
            return [name for name, checkbox in self.robots.items() if checkbox.isChecked()]

    def get_selected_urs(self):
        """Returns a list of selected UR prefixes."""
        ur_prefixes = []
        if self.ur10_l.isChecked():
            ur_prefixes.append("UR10_l")
        if self.ur10_r.isChecked():
            ur_prefixes.append("UR10_r")
        return ur_prefixes