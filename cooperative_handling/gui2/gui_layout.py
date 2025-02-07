import threading
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTableWidget, QCheckBox, QTableWidgetItem
from PyQt5.QtCore import QTimer, Qt
from ros_interface import start_status_update, open_rviz, run_compute_object_center, launch_drivers, quit_drivers, zero_ft_sensors, turn_on_wrench_controllers, turn_on_arm_controllers, turn_on_twist_controllers, enable_all_urs, update_ur_relative_to_object, launch_ros, move_to_initial_pose, turn_on_coop_admittance_controller
from relative_poses import RelativePoses
from ros_interface import ROSInterface

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()
        self.ros_interface = ROSInterface(self)
        self.setWindowTitle("Multi-Robot Demo")
        self.setGeometry(100, 100, 800, 500)
        
        self.workspace_name = "catkin_ws_recker"
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

        self.check_set_reference = QCheckBox("Set reference at runtime")
        self.check_set_reference.setChecked(True)  # Standardmäßig aktiviert

        
        # Timer for status updates
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(lambda: start_status_update(self))
        self.status_timer.start(30000)
        
        # Buttons for ROS Actions
        buttons = {
            "Check Status": lambda: start_status_update(self),
            "Open RVIZ": open_rviz,
            "Start Virtual Leader": lambda: launch_ros(self, "virtual_leader", "virtual_leader.launch"),
            "Start Virtual Object": lambda: launch_ros(self, "virtual_object", "virtual_object.launch"),
            "Compute Object Center": lambda: run_compute_object_center(self),
            "Zero F/T Sensors": lambda: zero_ft_sensors(self),
            "Turn on Wrench Controllers": lambda: turn_on_wrench_controllers(self),
            "Turn on Arm Controllers": lambda: turn_on_arm_controllers(self),
            "Turn on Twist Controllers": lambda: turn_on_twist_controllers(self),
            "Enable all URs": lambda: enable_all_urs(self),
            "Update UR relative to object": lambda: update_ur_relative_to_object(self),
            "Move to Initial Pose Left": lambda: move_to_initial_pose(self, "UR10_l"),
            "Move to Initial Pose Right": lambda: move_to_initial_pose(self, "UR10_r"),
            "Turn on Admittance Controller": lambda: turn_on_coop_admittance_controller(self),
            "Launch Drivers": lambda: launch_drivers(self),  # HIER GEÄNDERT!
            "Quit Drivers": lambda: quit_drivers()
        }

        for text, function in buttons.items():
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, f=function: f())  # HIER GEÄNDERT!
            left_layout.addWidget(btn)

        left_layout.addWidget(self.check_set_reference)
        
        main_layout.addLayout(left_layout)
        
        # Right Side (Table for Relative Poses)
        self.table = QTableWidget(8, 3)
        self.table.setHorizontalHeaderLabels(["X", "Y", "Z"])
        self.table.setVerticalHeaderLabels([
            "mur620a/UR10_l", "mur620a/UR10_r", "mur620b/UR10_l", "mur620b/UR10_r", 
            "mur620c/UR10_l", "mur620c/UR10_r", "mur620d/UR10_l", "mur620d/UR10_r"
        ])
        self.load_relative_poses()
        main_layout.addWidget(self.table)
        
        # Save Button
        self.btn_save_poses = QPushButton("Save Poses")
        self.btn_save_poses.setEnabled(False)  # Deaktiviert, bis Werte aktualisiert wurden
        self.btn_save_poses.clicked.connect(lambda: self.save_relative_poses())

        self.btn_update_poses = QPushButton("Update Poses")
        self.btn_update_poses.clicked.connect(self.ros_interface.update_poses)
        main_layout.addWidget(self.btn_update_poses)


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

    def save_relative_poses(self):
        """Sammelt die Werte aus der Tabelle und speichert sie."""
        poses = {}
        for row in range(self.table.rowCount()):
            row_label = self.table.verticalHeaderItem(row).text()
            poses[row_label] = [
                float(self.table.item(row, col).text()) if self.table.item(row, col) else 0.0
                for col in range(self.table.columnCount())
            ]
        
        relative_poses = RelativePoses()  # Erstelle eine Instanz der Klasse
        relative_poses.save_poses(poses)


    def load_relative_poses(self):
        """Lädt die gespeicherten Posen und setzt sie in die Tabelle ein."""
        relative_poses = RelativePoses()  # Instanz erstellen
        poses = relative_poses.load_poses()  # Geladene Posen als Dictionary

        for row in range(self.table.rowCount()):
            row_label = self.table.verticalHeaderItem(row).text()
            if row_label in poses:
                for col in range(self.table.columnCount()):
                    value = poses[row_label][col] if col < len(poses[row_label]) else 0.0
                    self.table.setItem(row, col, QTableWidgetItem(str(value)))
