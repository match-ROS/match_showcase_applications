import threading
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTableWidget, QCheckBox
from PyQt5.QtCore import QTimer, Qt
from ros_interface import start_status_update, open_rviz, run_compute_object_center, launch_drivers, quit_drivers, zero_ft_sensors, turn_on_wrench_controllers, turn_on_arm_controllers, turn_on_twist_controllers, enable_all_urs, update_ur_relative_to_object, launch_ros, move_to_initial_pose, turn_on_coop_admittance_controller
from relative_poses import RelativePoses

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
        ur_layout.addWidget(QLabel("Select URs:"))
        self.ur10_l = QCheckBox("UR10_l")
        self.ur10_r = QCheckBox("UR10_r")
        self.ur10_l.setChecked(True)
        self.ur10_r.setChecked(True)
        ur_layout.addWidget(self.ur10_l)
        ur_layout.addWidget(self.ur10_r)

        robot_ur_layout.addLayout(robot_layout)
        robot_ur_layout.addLayout(ur_layout)
        left_layout.addLayout(robot_ur_layout)
        
        # Timer for status updates
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(lambda: start_status_update(self))
        self.status_timer.start(30000)
        
        # Buttons for ROS Actions
        buttons = {
            "Check Status": lambda: start_status_update(self),
            "Open RVIZ": open_rviz,
            "Start Virtual Leader": lambda: launch_ros("virtual_leader", "virtual_leader.launch"),
            "Start Virtual Object": lambda: launch_ros("virtual_object", "virtual_object.launch"),
            "Compute Object Center": run_compute_object_center,
            "Zero F/T Sensors": zero_ft_sensors,
            "Turn on Wrench Controllers": turn_on_wrench_controllers,
            "Turn on Arm Controllers": turn_on_arm_controllers,
            "Turn on Twist Controllers": turn_on_twist_controllers,
            "Enable all URs": enable_all_urs,
            "Update UR relative to object": update_ur_relative_to_object,
            "Move to Initial Pose Left": lambda: move_to_initial_pose("UR10_l"),
            "Move to Initial Pose Right": lambda: move_to_initial_pose("UR10_r"),
            "Turn on Admittance Controller": turn_on_coop_admittance_controller
        }

        for text, function in buttons.items():
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, f=function: f())
            left_layout.addWidget(btn)
        
        main_layout.addLayout(left_layout)
        
        # Right Side (Table for Relative Poses)
        self.table = QTableWidget(8, 3)
        self.table.setHorizontalHeaderLabels(["X", "Y", "Z"])
        self.table.setVerticalHeaderLabels([
            "mur620a/UR10_l", "mur620a/UR10_r", "mur620b/UR10_l", "mur620b/UR10_r", 
            "mur620c/UR10_l", "mur620c/UR10_r", "mur620d/UR10_l", "mur620d/UR10_r"
        ])
        RelativePoses.load_poses(self.table)
        main_layout.addWidget(self.table)
        
        # Save Button
        self.btn_save_poses = QPushButton("Save Poses")
        self.btn_save_poses.clicked.connect(lambda: RelativePoses.save_poses(self.table))
        main_layout.addWidget(self.btn_save_poses)
        
        self.setLayout(main_layout)

    def get_selected_robots(self):
        return [name for name, checkbox in self.robots.items() if checkbox.isChecked()]

    def get_selected_urs(self):
        ur_prefixes = []
        if self.ur10_l.isChecked():
            ur_prefixes.append("UR10_l")
        if self.ur10_r.isChecked():
            ur_prefixes.append("UR10_r")
        return ur_prefixes
