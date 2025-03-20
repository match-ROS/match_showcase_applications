import threading
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTableWidget, QCheckBox, QTableWidgetItem, QGroupBox, QTabWidget, QDoubleSpinBox, QTextEdit
from PyQt5.QtCore import QTimer, Qt
from ros_interface import start_status_update, open_rviz, run_compute_object_center, launch_drivers, quit_drivers, zero_ft_sensors, turn_on_twist_controllers, launch_ros, move_to_initial_pose
from relative_poses import RelativePoses
from ros_interface import ROSInterface

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()
        self.ros_interface = ROSInterface(self)
        self.setWindowTitle("Multi-Robot Transport")
        self.setGeometry(100, 100, 2000, 1000)
        
        self.workspace_name = "catkin_ws_recker"
        main_layout = QHBoxLayout()
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.ros_interface.update_button_status)
        self.status_timer.start(5000)
        
        left_layout = QVBoxLayout()
        self.status_label = QLabel("Controller Status: Not Checked")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("border: 1px solid black; padding: 5px;")
        left_layout.addWidget(self.status_label)
        
        selection_group = QGroupBox("Robot Selection")
        selection_layout = QVBoxLayout()
        
        self.robots = {
            "mur620a": QCheckBox("mur620a"),
            "mur620b": QCheckBox("mur620b"),
            "mur620c": QCheckBox("mur620c"),
            "mur620d": QCheckBox("mur620d"),
        }
        for checkbox in self.robots.values():
            selection_layout.addWidget(checkbox)
        
        selection_group.setLayout(selection_layout)
        left_layout.addWidget(selection_group)
        
        setup_group = QGroupBox("Setup Functions")
        setup_layout = QVBoxLayout()
        setup_buttons = {
            "Check Status": lambda: start_status_update(self),
            "Launch Drivers": lambda: launch_drivers(self),
            "Quit Drivers": lambda: quit_drivers(),
            "Open RVIZ": open_rviz,
            "Start Virtual Leader": lambda: launch_ros(self, "virtual_leader", "virtual_leader.launch"),
            "Start Virtual Object": lambda: launch_ros(self, "virtual_object", "virtual_object.launch"),
            "Start Roscore": lambda: self.ros_interface.start_roscore(),
            "Start Mocap": lambda: self.ros_interface.start_mocap(),
            "Start Sync": lambda: self.ros_interface.start_sync(),
        }

        for text, function in setup_buttons.items():
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, f=function: f())
            btn.setStyleSheet("background-color: lightgray;")
            setup_layout.addWidget(btn)
        setup_group.setLayout(setup_layout)
        left_layout.addWidget(setup_group)
        
        main_layout.addLayout(left_layout)
        
        self.table = QTableWidget(5, 3)
        self.table.setHorizontalHeaderLabels(["X", "Y", "Yaw"])
        self.table.setVerticalHeaderLabels(["mur620a", "mur620b", "mur620c", "mur620d", "Virtual Object"])
        self.load_relative_poses()
        
        self.btn_save_poses = QPushButton("Save Poses")
        self.btn_save_poses.clicked.connect(lambda: self.save_relative_poses())
        
        self.btn_update_poses = QPushButton("Update Poses")
        self.btn_update_poses.clicked.connect(self.ros_interface.update_poses)
        
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.table)
        right_layout.addWidget(self.btn_save_poses)
        right_layout.addWidget(self.btn_update_poses)
        
        controller_group = QGroupBox("Controller Functions")
        controller_layout = QVBoxLayout()
        controller_buttons = {
            "Turn on Twist Controllers": lambda: turn_on_twist_controllers(self),
            "Move Object to Initial Pose": lambda: self.ros_interface.move_virtual_object_to_initial_pose(),
        }

        for text, function in controller_buttons.items():
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, f=function: f())
            controller_layout.addWidget(btn)
        
        controller_group.setLayout(controller_layout)
        right_layout.addWidget(controller_group)
        
        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)
        
    def load_relative_poses(self):
        relative_poses = RelativePoses()
        poses = relative_poses.load_poses()
        
        for row in range(self.table.rowCount()):
            row_label = self.table.verticalHeaderItem(row).text()
            if row_label in poses:
                for col in range(self.table.columnCount()):
                    value = poses[row_label][col] if col < len(poses[row_label]) else 0.0
                    self.table.setItem(row, col, QTableWidgetItem(str(value)))

    def save_relative_poses(self):
        poses = {}
        for row in range(self.table.rowCount()):
            row_label = self.table.verticalHeaderItem(row).text()
            poses[row_label] = [
                float(self.table.item(row, col).text()) if self.table.item(row, col) else 0.0
                for col in range(3)
            ]
        relative_poses = RelativePoses()
        relative_poses.save_poses(poses)

    def get_selected_robots(self):
        return [name for name, checkbox in self.robots.items() if checkbox.isChecked()]
