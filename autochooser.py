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
    QLabel, QPushButton, QFrame, QSizePolicy, QLineEdit
)
from PyQt6.QtCore import QTimer, QCoreApplication, Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

# --- Auto Route Graph Definition ---
ROOTS = ["Left Start", "Center Start", "Right Start"]

EDGES = [
    ("Left Start", "LeftShootCenter"),
    ("Left Start", "LeftShootLeft"),
    ("Left Start", "SimpleLeftSweep"),
    ("Center Start", "CenterShootCenter"),
    ("Right Start", "RightShootCenter"),
    ("Right Start", "RightShootRight"),
    ("Right Start", "SimpleRightSweep"),
    ("LeftShootCenter", "SweepAroundLeft"),
    ("LeftShootCenter", "SweepAroundRight"),
    ("CenterShootCenter", "SweepAroundLeft"),
    ("CenterShootCenter", "SweepAroundRight"),
    ("RightShootCenter", "SweepAroundLeft"),
    ("RightShootCenter", "SweepAroundRight"),
    ("SweepAroundRight", "LeftInShoot"),
    ("SweepAroundLeft", "RightInShoot"),
    ("SimpleLeftSweep", "RightInShoot"),
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
    "RightInShoot": (6, 4),
    "LeftInShoot": (6, 2),
}

NODE_LABELS = {
    "Left Start": "Left Start",
    "Center Start": "Center Start",
    "Right Start": "Right Start",
    "SimpleLeftSweep": "Simple Left Sweep",
    "LeftShootCenter": "Left Shoot Center",
    "LeftShootLeft": "Left Shoot Left",
    "CenterShootCenter": "Center Shoot Center",
    "RightShootCenter": "Right Shoot Center",
    "SimpleRightSweep": "Simple Right Sweep",
    "RightShootRight": "Right Shoot Right",
    "SweepAroundLeft": "Sweep Around Left",
    "SweepAroundRight": "Sweep Around Right",
    "LeftInShoot": "Left In Shoot",
    "RightInShoot": "Right In Shoot",
}

NODE_CODES = {
    "Left Start": "LS",
    "Center Start": "CS",
    "Right Start": "RS",
    "SimpleLeftSweep": "SL",
    "LeftShootCenter": "LC",
    "LeftShootLeft": "LL",
    "CenterShootCenter": "CC",
    "RightShootCenter": "RC",
    "SimpleRightSweep": "SR",
    "RightShootRight": "RR",
    "SweepAroundLeft": "WL",
    "SweepAroundRight": "WR",
    "LeftInShoot": "LI",
    "RightInShoot": "RI",
}

CODE_TO_NODE = {code: node for node, code in NODE_CODES.items()}
ADJACENCY = {node: [] for node in POSITIONS}
for start_node, end_node in EDGES:
    ADJACENCY[start_node].append(end_node)


def format_route(route):
    return " -> ".join(NODE_LABELS.get(node, node) for node in route)


def format_route_code(route):
    if not route:
        return "not selected"
    return "-".join(NODE_CODES[node] for node in route)


def parse_route_code(route_code):
    normalized = route_code.strip().upper().replace(" ", "")
    if not normalized:
        raise ValueError("Enter a route code.")

    parts = [part for part in normalized.split("-") if part]
    if not parts:
        raise ValueError("Enter a route code.")

    try:
        path = [CODE_TO_NODE[part] for part in parts]
    except KeyError as exc:
        raise ValueError(f"Unknown route code segment: {exc.args[0]}") from exc

    if path[0] not in ROOTS:
        raise ValueError("Route code must start with a start position.")

    for current, next_node in zip(path, path[1:]):
        if next_node not in ADJACENCY.get(current, []):
            raise ValueError(f"Invalid transition in route code: {NODE_CODES[current]}-{NODE_CODES[next_node]}")

    return path

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
QLineEdit {
    background-color: #444444;
    color: white;
    border: 1px solid #555555;
    border-radius: 5px;
    padding: 8px;
    font-family: Arial, sans-serif;
}
"""

class AutoSelectorGUI(QWidget):
    def __init__(self, ip_arg):
        super().__init__()
        
        self.setObjectName("MainGUI")
        self.setWindowTitle("Trinity AutoChooser")
        self.resize(1000, 650)

        self.current_path = []
        self.path_lines = []
        self.path_scatters = []

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
        title = QLabel("Trinity AutoChooser")
        title.setObjectName("Title")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        controls_layout.addWidget(title)

        # Question / Info Label
        self.question_label = QLabel("Select a starting position:")
        self.question_label.setObjectName("Question")
        self.question_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        controls_layout.addWidget(self.question_label)

        self.preview_label = QLabel("Route: not selected")
        self.preview_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        controls_layout.addWidget(self.preview_label)

        self.code_label = QLabel("Code: not selected")
        self.code_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        controls_layout.addWidget(self.code_label)

        restore_row = QHBoxLayout()
        self.code_input = QLineEdit()
        self.code_input.setPlaceholderText("Restore route code, e.g. LS-LC-WR-LI")
        self.code_input.returnPressed.connect(self.restore_path_from_code)
        restore_row.addWidget(self.code_input)

        self.restore_btn = QPushButton("Restore Code")
        self.restore_btn.clicked.connect(self.restore_path_from_code)
        restore_row.addWidget(self.restore_btn)
        controls_layout.addLayout(restore_row)

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

        self.render_current_step()

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
            self.ax.text(
                x,
                y + 0.3,
                NODE_LABELS.get(node, node),
                color='white',
                ha='center',
                fontsize=9,
                alpha=0.7
            )

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

    def render_current_step(self):
        self._clear_buttons()
        self.preview_label.setText(
            f"Route: {format_route(self.current_path) if self.current_path else 'not selected'}"
        )
        self.code_label.setText(f"Code: {format_route_code(self.current_path)}")
        self.code_input.setText("" if not self.current_path else format_route_code(self.current_path))

        if self.current_path:
            send_btn = QPushButton("Send to Robot")
            send_btn.setObjectName("ActionBtn")
            send_btn.clicked.connect(self.finish_path)
            self.buttons_layout.addWidget(send_btn)

        if not self.current_path:
            self.question_label.setText("Select a starting position:")
            next_nodes = ROOTS
        else:
            current_node = self.current_path[-1]
            next_nodes = ADJACENCY.get(current_node, [])
            if next_nodes:
                self.question_label.setText(
                    f"Current node: {NODE_LABELS[current_node]}\nChoose the next route or send to stop here."
                )
            else:
                self.question_label.setText(
                    f"Current node: {NODE_LABELS[current_node]}\nNo further routes are available."
                )

        for node in next_nodes:
            btn = QPushButton(NODE_LABELS.get(node, node))
            btn.setObjectName("ActionBtn")
            btn.clicked.connect(lambda checked, next_node=node: self.choose_node(next_node))
            self.buttons_layout.addWidget(btn)

    def choose_node(self, node):
        self.buttons_container.setEnabled(False)
        self.restart_btn.setEnabled(False)
        self.restore_btn.setEnabled(False)
        self._append_route_node(node)
        self.buttons_container.setEnabled(True)
        self.restart_btn.setEnabled(True)
        self.restore_btn.setEnabled(True)
        self.render_current_step()

    def _append_route_node(self, node):
        if self.current_path:
            self.animate_step(self.current_path[-1], node)

        x, y = POSITIONS[node]
        scat = self.ax.scatter(
            x,
            y,
            color='#ffcc00',
            s=180,
            zorder=3,
            edgecolors='white',
            linewidths=2
        )
        self.path_scatters.append(scat)
        self.canvas.draw()
        self.current_path.append(node)

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
        display_label = f"{format_route(self.current_path)} -> Stop"
        
        # Publish to NT4
        self.selected_pub.set(auto_label)
        print(f"[NT] Published to 'SmartDashboard/Auto Choices/selected': {auto_label}")
        
        self.question_label.setText(f"Sent to Robot!\n\n{display_label}")
        self._clear_buttons()
        send_again_btn = QPushButton("Send Again")
        send_again_btn.setObjectName("ActionBtn")
        send_again_btn.clicked.connect(self.finish_path)
        self.buttons_layout.addWidget(send_again_btn)

    def restart_path(self):
        self.current_path.clear()

        for line in self.path_lines:
            line.remove()
        for scat in self.path_scatters:
            scat.remove()
            
        self.path_lines.clear()
        self.path_scatters.clear()
        self.canvas.draw()

        self.render_current_step()

    def restore_path_from_code(self):
        try:
            restored_path = parse_route_code(self.code_input.text())
        except ValueError as exc:
            self.question_label.setText(f"Could not restore route.\n\n{exc}")
            return

        self.restart_path()
        self.buttons_container.setEnabled(False)
        self.restart_btn.setEnabled(False)
        self.restore_btn.setEnabled(False)
        for node in restored_path:
            self._append_route_node(node)
        self.buttons_container.setEnabled(True)
        self.restart_btn.setEnabled(True)
        self.restore_btn.setEnabled(True)
        self.render_current_step()
        self.question_label.setText("Route restored from code.\nSend to Robot or continue extending it.")

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
