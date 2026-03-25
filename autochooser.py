# /// script
# requires-python = ">=3.10"
# dependencies =[
#     "PyQt6",
#     "matplotlib",
#     "robotpy"
# ]
# ///

import sys
import time
import argparse
import ntcore
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QPushButton, QFrame, QSizePolicy
)
from PyQt6.QtCore import QTimer, QCoreApplication, Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

# --- Auto Route Graph Definition ---
ROOTS =["Left Start", "Center Start", "Right Start"]

EDGES =[
    ("Center Start", "CenterShootCenter"),
    ("CenterShootCenter", "SweepAroundLeft"),
    ("CenterShootCenter", "SweepAroundRight"),
    ("SweepAroundRight", "LeftInShoot"),
    ("SweepAroundLeft", "RightInShoot"),
    ("Left Start", "LeftShootCenter"),
    ("LeftShootCenter", "SweepAroundLeft"),
    ("LeftShootCenter", "SweepAroundRight"),
    ("Right Start", "RightShootCenter"),
    ("RightShootCenter", "SweepAroundLeft"),
    ("RightShootCenter", "SweepAroundRight"),
    ("Left Start", "SimpleLeftSweep"),
    ("SimpleLeftSweep", "RightInShoot"),
    ("Left Start", "LeftShootLeft"),
    ("Right Start", "RightShootRight"),
    ("Right Start", "SimpleRightSweep")
]

POSITIONS = {
    "Left Start": (0, 6),
    "Center Start": (0, 3),
    "Right Start": (0, 0),

    "LeftShootLeft": (2, 7.5),
    "SimpleLeftSweep": (2, 5.5),
    "LeftShootCenter": (2, 4.5),
    "CenterShootCenter": (2, 3),
    "RightShootCenter": (2, 1.5),
    "SimpleRightSweep": (2, 0.5),
    "RightShootRight": (2, -1.5),

    "SweepAroundLeft": (4, 4),
    "SweepAroundRight": (4, 2),

    "LeftInShoot": (6, 5),
    "RightInShoot": (6, 1),
}

# Build adjacency list
ADJACENCY = {node:[] for node in POSITIONS}
for u, v in EDGES:
    ADJACENCY[u].append(v)

# --- Qt Style Sheet (Dark Theme + CyberCheese Colors) ---
STYLESHEET = """
QWidget#MainGUI {
    background-color: #2b2b2b;
}
QFrame#ControlsFrame {
    background-color: #333333;
    border-radius: 10px;
}
QWidget#ButtonsContainer {
    background-color: transparent;
}
QLabel {
    background-color: transparent;
    color: #ffffff;
    font-family: Arial, sans-serif;
}
QPushButton {
    background-color: #444444;
    color: white;
    border: none;
    border-radius: 5px;
    padding: 12px;
    font-size: 14px;
    font-weight: bold;
    font-family: Arial, sans-serif;
}
QPushButton:hover {
    background-color: #555555;
}
QPushButton#ActionBtn {
    background-color: #ffcc00;
    color: black;
}
QPushButton#ActionBtn:hover {
    background-color: #e6b800;
}
QPushButton#StopBtn {
    background-color: #d9534f;
    color: white;
}
QPushButton#StopBtn:hover {
    background-color: #c9302c;
}
QLabel#Title {
    font-size: 22px;
    font-weight: bold;
    color: #ffcc00;
}
QLabel#Status {
    font-size: 14px;
    font-weight: bold;
}
QLabel#Question {
    font-size: 16px;
}
"""

class AutoSelectorGUI(QWidget):
    def __init__(self, ip_arg):
        super().__init__()
        
        self.setObjectName("MainGUI")
        self.setWindowTitle("Trinity AutoChooser")
        self.resize(1000, 650)
        
        self.current_path =[]
        self.path_lines =[]
        self.path_scatters =[]

        # --- NetworkTables Setup ---
        self.nt_inst = ntcore.NetworkTableInstance.getDefault()
        self.nt_inst.startClient4("TrinityAutoChooser")
        
        # Completely native FRC compliant C++ Server handling (solves the tuple/array error)
        if ip_arg:
            self.nt_inst.setServer(ip_arg)
        else:
            self.nt_inst.setServerTeam(8847)
            
        self.chooser_table = self.nt_inst.getTable("SmartDashboard/Auto Choices")
        # Removing PubSubOptions to avoid an obscure `robotpy` version mismatch issue. Standard publish() is safer here.
        self.selected_pub = self.chooser_table.getStringTopic("selected").publish()

        self._build_ui()
        
        # Start connection heartbeat timer
        self.nt_timer = QTimer(self)
        self.nt_timer.timeout.connect(self._check_nt_status)
        self.nt_timer.start(1000)
        self._check_nt_status()

    def _build_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # --- Left Panel: Controls & Questions ---
        self.controls_frame = QFrame()
        self.controls_frame.setObjectName("ControlsFrame")
        controls_layout = QVBoxLayout(self.controls_frame)
        controls_layout.setContentsMargins(20, 20, 20, 20)
        controls_layout.setSpacing(15)

        # Connection Status
        self.status_label = QLabel("Disconnected")
        self.status_label.setObjectName("Status")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        controls_layout.addWidget(self.status_label)

        # Title
        title = QLabel("Auto Route Builder")
        title.setObjectName("Title")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        controls_layout.addWidget(title)

        # Question / Info Label
        self.question_label = QLabel("Select a starting position:")
        self.question_label.setObjectName("Question")
        self.question_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        controls_layout.addWidget(self.question_label)

        # Dynamic Buttons Layout Container
        self.buttons_container = QWidget()
        self.buttons_container.setObjectName("ButtonsContainer")
        self.buttons_layout = QVBoxLayout(self.buttons_container)
        self.buttons_layout.setContentsMargins(0, 10, 0, 10)
        self.buttons_layout.setSpacing(10)
        controls_layout.addWidget(self.buttons_container)

        controls_layout.addStretch()

        # Restart Button
        self.restart_btn = QPushButton("Restart Path")
        self.restart_btn.clicked.connect(self.restart_path)
        controls_layout.addWidget(self.restart_btn)

        # --- Right Panel: Matplotlib Visualization ---
        self.plot_frame = QFrame()
        plot_layout = QVBoxLayout(self.plot_frame)
        plot_layout.setContentsMargins(0, 0, 0, 0)
        
        self._setup_matplotlib(plot_layout)

        main_layout.addWidget(self.controls_frame, 1)
        main_layout.addWidget(self.plot_frame, 3)

        self.ask_next()

    def _setup_matplotlib(self, parent_layout):
        self.fig, self.ax = plt.subplots(figsize=(7, 6))
        self.fig.patch.set_facecolor('#2b2b2b')
        self.ax.set_facecolor('#2b2b2b')
        self.ax.axis('off')
        self.fig.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)

        for u, v in EDGES:
            x_vals =[POSITIONS[u][0], POSITIONS[v][0]]
            y_vals = [POSITIONS[u][1], POSITIONS[v][1]]
            self.ax.plot(x_vals, y_vals, color='gray', alpha=0.3, linewidth=2, zorder=1)

        for node, (x, y) in POSITIONS.items():
            self.ax.scatter(x, y, color='gray', s=80, alpha=0.5, zorder=2)
            self.ax.text(x, y + 0.3, node, color='white', ha='center', fontsize=9, alpha=0.7)

        self.canvas = FigureCanvasQTAgg(self.fig)
        self.canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.canvas.updateGeometry()
        parent_layout.addWidget(self.canvas)

    def _check_nt_status(self):
        if self.nt_inst.isConnected():
            self.status_label.setText("Connected to NT")
            self.status_label.setStyleSheet("color: #00ff00;")
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: #ff4444;")

    def _clear_buttons(self):
        while self.buttons_layout.count():
            child = self.buttons_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def ask_next(self):
        self._clear_buttons()

        if not self.current_path:
            self.question_label.setText("Select a starting position:")
            for root in ROOTS:
                btn = QPushButton(root)
                btn.setObjectName("ActionBtn")
                btn.clicked.connect(lambda checked, r=root: self.choose_node(r))
                self.buttons_layout.addWidget(btn)
        else:
            last_node = self.current_path[-1]
            options = ADJACENCY.get(last_node,[])
            self.question_label.setText(f"Current: {last_node}\nChoose next step:")

            for opt in options:
                btn = QPushButton(opt)
                btn.setObjectName("ActionBtn")
                btn.clicked.connect(lambda checked, o=opt: self.choose_node(o))
                self.buttons_layout.addWidget(btn)

            stop_btn = QPushButton("Stop Here")
            stop_btn.setObjectName("StopBtn")
            stop_btn.clicked.connect(self.finish_path)
            self.buttons_layout.addWidget(stop_btn)

    def choose_node(self, node):
        self.buttons_container.setEnabled(False)
        self.restart_btn.setEnabled(False)

        if self.current_path:
            self.animate_step(self.current_path[-1], node)
        
        x, y = POSITIONS[node]
        scat = self.ax.scatter(x, y, color='#ffcc00', s=180, zorder=3, edgecolors='white', linewidths=2)
        self.path_scatters.append(scat)
        self.canvas.draw()

        self.current_path.append(node)
        
        self.buttons_container.setEnabled(True)
        self.restart_btn.setEnabled(True)
        self.ask_next()

    def animate_step(self, start_node, end_node):
        x1, y1 = POSITIONS[start_node]
        x2, y2 = POSITIONS[end_node]
        steps = 15
        line, = self.ax.plot([],[], color='#ffcc00', linewidth=5, zorder=2)
        
        for i in range(1, steps + 1):
            t = i / steps
            cx, cy = x1 + t*(x2-x1), y1 + t*(y2-y1)
            line.set_data([x1, cx], [y1, cy])
            self.canvas.draw()
            QCoreApplication.processEvents() 
            time.sleep(0.01)
            
        self.path_lines.append(line)

    def finish_path(self):
        # Format explicitly identical to Java backend
        final_list = self.current_path + ["Stop"]
        auto_label = " -> ".join(final_list)
        
        # Publish to NT4
        self.selected_pub.set(auto_label)
        print(f"[NT] Published to 'SmartDashboard/Auto Choices/selected': {auto_label}")
        
        self.question_label.setText(f"Sent to Robot!\n\n{auto_label}")
        self._clear_buttons()

    def restart_path(self):
        self.current_path.clear()
        
        for line in self.path_lines:
            line.remove()
        for scat in self.path_scatters:
            scat.remove()
            
        self.path_lines.clear()
        self.path_scatters.clear()
        self.canvas.draw()
        
        self.ask_next()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CyberCheese Auto Selector GUI")
    parser.add_argument("--ip", type=str, default="", help="IP address to connect to (e.g. 127.0.0.1 for sim)")
    # Using parse_known_args because PyQt5/PyQt6 also uses some command line args natively under the hood
    args, unknown = parser.parse_known_args()

    app = QApplication(sys.argv)
    app.setStyleSheet(STYLESHEET)
    
    gui = AutoSelectorGUI(args.ip)
    gui.show()
    
    sys.exit(app.exec())