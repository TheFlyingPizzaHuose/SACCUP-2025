#Developed by Ryan Santiago for visualization of telemetry data


import sys, os, time, serial, numpy as np, cv2
from PyQt5.QtGui import QImage, QPixmap, QFont, QPainter, QColor, QBrush, QConicalGradient, QFontDatabase
from PyQt5.QtWidgets import (
    QApplication, QLabel, QVBoxLayout, QWidget, QHBoxLayout,
    QTextEdit, QGridLayout, QPushButton, QInputDialog, QComboBox,
    QFileDialog, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer, QRect, pyqtSignal, QThread, QUrl
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from collections import deque
import json
import csv
import re
from datetime import datetime

DEMO_MODE = False
PORT_NAME = "COM8"

class LivePlotWidget(QWidget):
    def __init__(self, title, max_points=300, parent=None):
        super().__init__(parent)
        self.figure = Figure(figsize=(4, 3), facecolor='white')
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.title = title
        self.max_points = max_points
        self.x_data = deque(maxlen=max_points)
        self.y_data = deque(maxlen=max_points)
        self.line, = self.ax.plot([], [], color='black')
        self.init_plot()

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def init_plot(self):
        self.ax.set_title(self.title, color='black', pad=10)
        self.ax.set_facecolor('white')
        self.ax.tick_params(axis='x', colors='black', labelsize=9)
        self.ax.tick_params(axis='y', colors='black', labelsize=9)
        self.ax.xaxis.label.set_color('black')
        self.ax.yaxis.label.set_color('black')
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel(self.title)
        self.figure.subplots_adjust(left=0.15, right=0.95, top=0.88, bottom=0.25)

    def update_plot(self, x, y):
        self.x_data.append(x)
        self.y_data.append(y)
        self.line.set_data(self.x_data, self.y_data)
        self.ax.set_xlim(max(0, self.x_data[0]), x + 1)
        min_y, max_y = min(self.y_data, default=0), max(self.y_data, default=1)
        margin = (max_y - min_y) * 0.1 if max_y != min_y else 1
        self.ax.set_ylim(min_y - margin, max_y + margin)
        self.canvas.draw()

class TeensyReader(QThread):
    data_received = pyqtSignal(str)
    def __init__(self, port=PORT_NAME, baud=115200):
        super().__init__()
        self.port, self.baud = port, baud
        self.running, self.connected = True, False
        self.serial_conn = None
    
    def run(self):
        if DEMO_MODE:
            start_time = time.time()
            base_lat, base_lon = 38.6, -90.2  # Starting coordinates
            while self.running:
                t = time.time() - start_time
                altitude = 1000 + 5000 * np.sin(0.01 * t) + np.random.normal(0, 50)
                velocity = 200 + 100 * np.sin(0.03 * t) + np.random.normal(0, 5)
                orient_x = 10 * np.sin(0.1 * t)
                orient_y = 10 * np.cos(0.1 * t)
                
                # Simulate GPS movement (rocket drifts during flight)
                lat_offset = 0.001 * np.sin(0.005 * t) + np.random.normal(0, 0.0001)
                lon_offset = 0.001 * np.cos(0.005 * t) + np.random.normal(0, 0.0001)
                current_lat = base_lat + lat_offset
                current_lon = base_lon + lon_offset
                
                event = np.random.choice(["LIFTOFF", "BURNOUT", "APOGEE", "DROGUE", "MAIN", "LANDED"])
                data = (
                    f"T+: {int(t)} ALT: {altitude:.1f} VEL: {velocity:.1f} "
                    f"LAT: {current_lat:.6f} LON: {current_lon:.6f} "
                    f"OREINT_X: {orient_x:.1f} OREINT_Y: {orient_y:.1f} EVENT: {event}"
                )
                self.data_received.emit(data)
                time.sleep(0.1)
            return
        
        while self.running:
            try:
                if not self.connected:
                    self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
                    self.connected = True
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line:
                    self.data_received.emit(line)
            except:
                self.connected = False
                time.sleep(1)
    
    def send_command(self, cmd):
        """Send command to Teensy"""
        if DEMO_MODE:
            print(f"[Demo Mode] Would send: {cmd}")
            return True
        elif self.connected and self.serial_conn:
            try:
                self.serial_conn.write((cmd + "\n").encode('utf-8'))
                print(f"[Sent] {cmd}")
                return True
            except Exception as e:
                print(f"Failed to send command: {e}")
                return False
        return False
    
    def stop(self):
        self.running = False
        if self.connected and self.serial_conn:
            self.serial_conn.close()

class DialWidget(QWidget):
    def __init__(self, title, max_value, parent=None):
        super().__init__(parent)
        self.setFixedSize(150, 150)
        self.value = 0
        self.max_value = max_value
        self.title = title

    def set_value(self, value):
        self.value = value
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRect(10, 10, 130, 130)
        painter.setPen(QColor(255, 255, 255))
        painter.setBrush(QColor(0, 0, 0, 180))
        painter.drawEllipse(rect)

        gradient = QConicalGradient(rect.center(), 90)
        if self.value < 0.3 * self.max_value:
            gradient.setColorAt(0, QColor(0, 255, 0))
            gradient.setColorAt(1, QColor(0, 255, 0))
        elif self.value < 0.7 * self.max_value:
            gradient.setColorAt(0, QColor(255, 255, 0))
            gradient.setColorAt(1, QColor(255, 255, 0))
        else:
            gradient.setColorAt(0, QColor(255, 0, 0))
            gradient.setColorAt(1, QColor(255, 0, 0))

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(gradient))
        arc_angle = int(-self.value / self.max_value * 180 * 16)
        painter.drawPie(rect, 90 * 16, arc_angle)

        painter.setPen(QColor(255, 255, 255))
        painter.setFont(QFont("Orbitron", 16, QFont.Bold))
        painter.drawText(rect, Qt.AlignCenter, f"{int(self.value)}")

        parts = self.title.split(" ")
        if len(parts) > 1:
            main = parts[0]
            unit = " ".join(parts[1:])
            painter.setFont(QFont("Orbitron", 10, QFont.Bold))
            painter.drawText(rect.adjusted(0, 30, 0, 0), Qt.AlignHCenter | Qt.AlignBottom, main)
            painter.setFont(QFont("Orbitron", 8, QFont.Bold))
            painter.drawText(rect.adjusted(0, 45, 0, 0), Qt.AlignHCenter | Qt.AlignBottom, unit)
        else:
            painter.setFont(QFont("Orbitron", 10, QFont.Bold))
            painter.drawText(rect, Qt.AlignBottom | Qt.AlignHCenter, self.title)

class VideoThread(QThread):
    frame_ready = pyqtSignal(object, int)
    def __init__(self, camera_id):
        super().__init__()
        self.camera_id = camera_id
        self.running = True
    
    def run(self):
        if DEMO_MODE:
            while self.running:
                frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                self.frame_ready.emit(frame, self.camera_id)
                time.sleep(0.03)
            return
        
        if self.camera_id == 4:  # assuming 4 is OBS
            cap = cv2.VideoCapture(self.camera_id, cv2.CAP_DSHOW)
        else:  # for DroidCam or other virtual cams
            cap = cv2.VideoCapture(self.camera_id)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        while self.running:
            ret, frame = cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.frame_ready.emit(frame, self.camera_id)
            else:
                # If camera fails, show placeholder
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, f"Camera {self.camera_id} Offline", (50, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                self.frame_ready.emit(frame, self.camera_id)
            time.sleep(0.03)
        cap.release()
    
    def stop(self):
        self.running = False

class RecordingManager:
    def __init__(self, main_window):
        self.main_window = main_window
        self.is_recording = False
        self.video_writer = None
        self.telemetry_file = None
        self.telemetry_writer = None
        self.recording_start_time = None
        
    def start_recording(self):
        if not self.is_recording:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Setup video recording
            video_filename = f"telemetry_recording_{timestamp}.avi"
            fourcc = cv2.VideoWriter_fourcc(*'XVID')

            
            # Get screen size for recording
            screen_size = (self.main_window.width(), self.main_window.height())
            self.video_writer = cv2.VideoWriter(video_filename, fourcc, 10.0, screen_size)
            
            # Setup telemetry CSV recording
            telemetry_filename = f"telemetry_data_{timestamp}.csv"
            self.telemetry_file = open(telemetry_filename, 'w', newline='')
            fieldnames = ['timestamp', 'T+', 'Altitude', 'Velocity', 'Latitude', 'Longitude', 
                         'Orientation_X', 'Orientation_Y', 'Event']
            self.telemetry_writer = csv.DictWriter(self.telemetry_file, fieldnames=fieldnames)
            self.telemetry_writer.writeheader()
            
            self.is_recording = True
            self.recording_start_time = time.time()
            
            return True, f"Recording started: {video_filename}, {telemetry_filename}"
        return False, "Already recording"
    
    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
            
            if self.telemetry_file:
                self.telemetry_file.close()
                self.telemetry_file = None
                self.telemetry_writer = None
            
            return True, "Recording stopped"
        return False, "Not currently recording"
    
    def record_frame(self, pixmap):
        if self.is_recording and self.video_writer:
            try:
                # Convert QPixmap to numpy array for OpenCV
                image = pixmap.toImage()
                width = image.width()
                height = image.height()
                
                ptr = image.bits()
                ptr.setsize(image.byteCount())
                arr = np.array(ptr).reshape(height, width, 4)  # 4 channels for RGBA
                
                # Convert RGBA to BGR for OpenCV
                bgr_image = cv2.cvtColor(arr[:, :, :3], cv2.COLOR_RGB2BGR)
                
                self.video_writer.write(bgr_image)
            except Exception as e:
                print(f"Error recording frame: {e}")
    
    def record_telemetry(self, telemetry_data):
        if self.is_recording and self.telemetry_writer:
            try:
                row = {
                    'timestamp': datetime.now().isoformat(),
                    'T+': telemetry_data.get('T+', 0),
                    'Altitude': telemetry_data.get('Altitude', 0),
                    'Velocity': telemetry_data.get('Velocity', 0),
                    'Latitude': telemetry_data.get('Latitude', 0),
                    'Longitude': telemetry_data.get('Longitude', 0),
                    'Orientation_X': telemetry_data.get('Orientation X', 0),
                    'Orientation_Y': telemetry_data.get('Orientation Y', 0),
                    'Event': telemetry_data.get('Event', 'None')
                }
                self.telemetry_writer.writerow(row)
                self.telemetry_file.flush()  # Ensure data is written immediately
            except Exception as e:
                print(f"Error recording telemetry: {e}")

class CommandWindow(QWidget):
    def __init__(self, teensy_reader):
        super().__init__()
        self.teensy_reader = teensy_reader
        self.setWindowTitle("Rocket Command Center")
        self.setStyleSheet("""
            QWidget {
                background-color: #1a1a1a;
                color: #00ff00;
                font-family: 'Courier New', monospace;
            }
            QComboBox {
                background-color: #2a2a2a;
                border: 2px solid #00ff00;
                padding: 8px;
                font-size: 14px;
                border-radius: 5px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #00ff00;
            }
            QPushButton {
                background-color: #2a2a2a;
                border: 2px solid #00ff00;
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #3a3a3a;
            }
            QPushButton:pressed {
                background-color: #00ff00;
                color: black;
            }
            QTextEdit {
                background-color: #0a0a0a;
                border: 2px solid #00ff00;
                padding: 10px;
                font-size: 12px;
                border-radius: 5px;
            }
        """)
        self.setGeometry(50, 50, 500, 400)
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("STARLANCE COMMAND CENTER")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #ffff00; padding: 10px;")
        layout.addWidget(title)
        
        # Connection status
        self.connection_status = QLabel("● Connection Status: CHECKING...")
        self.connection_status.setStyleSheet("font-size: 14px; padding: 5px;")
        layout.addWidget(self.connection_status)
        
        # Command selection
        cmd_label = QLabel("Select Command:")
        cmd_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px;")
        layout.addWidget(cmd_label)
        
        self.command_box = QComboBox()
        self.command_box.addItems([
            "1 - Low Power Mode",
            "2 - Normal Power Mode", 
            "3 - Video On",
            "4 - Video Off",
            "6 - Toggle Power/Sat Status",
            "7 - Arm for Launch",
            "h - Help"
        ])
        layout.addWidget(self.command_box)
        
        # Send button
        self.send_button = QPushButton("🚀 SEND COMMAND 🚀")
        self.send_button.clicked.connect(self.send_command)
        layout.addWidget(self.send_button)
        
        # Command log
        log_label = QLabel("Command Log:")
        log_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px;")
        layout.addWidget(log_label)
        
        self.command_log = QTextEdit()
        self.command_log.setReadOnly(True)
        self.command_log.setMaximumHeight(150)
        layout.addWidget(self.command_log)
        
        # Emergency stop button
        self.emergency_button = QPushButton("🛑 EMERGENCY STOP 🛑")
        self.emergency_button.setStyleSheet("""
            QPushButton {
                background-color: #aa0000;
                border: 3px solid #ff0000;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 15px;
            }
            QPushButton:hover {
                background-color: #cc0000;
            }
            QPushButton:pressed {
                background-color: #ff0000;
            }
        """)
        self.emergency_button.clicked.connect(self.emergency_stop)
        layout.addWidget(self.emergency_button)
        
        self.setLayout(layout)
        
        # Timer to update connection status
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_connection_status)
        self.status_timer.start(1000)  # Update every second

    def send_command(self):
        selected_item = self.command_box.currentText()
        cmd = selected_item.split(" - ")[0]  # Extract just the command part
        
        # Log the command attempt
        timestamp = time.strftime("%H:%M:%S")
        self.command_log.append(f"[{timestamp}] Sending: {selected_item}")
        
        # Send command via teensy reader
        success = self.teensy_reader.send_command(cmd)
        
        if success:
            self.command_log.append(f"[{timestamp}] ✓ Command sent successfully")
        else:
            self.command_log.append(f"[{timestamp}] ✗ Failed to send command")
        
        # Auto-scroll to bottom
        self.command_log.verticalScrollBar().setValue(
            self.command_log.verticalScrollBar().maximum()
        )

    def emergency_stop(self):
        timestamp = time.strftime("%H:%M:%S")
        self.command_log.append(f"[{timestamp}] 🛑 EMERGENCY STOP ACTIVATED 🛑")
        
        # Send emergency stop command (you may want to define a specific command for this)
        success = self.teensy_reader.send_command("STOP")
        
        if success:
            self.command_log.append(f"[{timestamp}] ✓ Emergency stop sent")
        else:
            self.command_log.append(f"[{timestamp}] ✗ Failed to send emergency stop")

    def update_connection_status(self):
        if self.teensy_reader.connected:
            self.connection_status.setText("● Connection Status: CONNECTED")
            self.connection_status.setStyleSheet("font-size: 14px; color: #00ff00; padding: 5px;")
        else:
            self.connection_status.setText("● Connection Status: DISCONNECTED")
            self.connection_status.setStyleSheet("font-size: 14px; color: #ff0000; padding: 5px;")

class TelemetryGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rocket Telemetry GUI")
        self.setStyleSheet("background-color: black;")
        self.setGeometry(100, 100, 1280, 720)

        # Try to load custom font
        font_path = os.path.join(os.getcwd(), "Orbitron-VariableFont_wght.ttf")
        if os.path.exists(font_path):
            font_id = QFontDatabase.addApplicationFont(font_path)
            if font_id != -1:
                font_family = QFontDatabase.applicationFontFamilies(font_id)[0]
                QApplication.setFont(QFont(font_family, 12))

        self.telemetry_data = {
            "T+": 0, "Altitude": 0, "Velocity": 0,
            "Latitude": 0.0, "Longitude": 0.0,
            "Orientation X": 0, "Orientation Y": 0,
            "Event": "None"
        }
        
        # Initialize recording manager
        self.recording_manager = RecordingManager(self)
        
        # Event flash variables
        self.last_event = ""
        self.event_flash_timer = QTimer()
        self.event_flash_timer.timeout.connect(self.clear_event_flash)
        
        self.initUI()
        self.setupThreads()
        self.start_time = time.time()
        
        # Create command window with reference to teensy reader
        self.cmd_window = CommandWindow(self.teensy_thread)
        self.cmd_window.show()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_telemetry_display)
        self.timer.start(100)
        
        # Timer for recording frames
        self.recording_timer = QTimer()
        self.recording_timer.timeout.connect(self.capture_frame_for_recording)
        self.recording_timer.start(100)  # 10 FPS recording

    def initUI(self):
        layout = QVBoxLayout(self)

        # Logo and recording controls row
        header_layout = QHBoxLayout()
        
        # Logo
        self.logo_label = QLabel()
        logo_path = r"C:\Users\rayit\OneDrive\Documentos\IREC 2025\Groundstation\logo.png"
        if os.path.exists(logo_path):
            pixmap = QPixmap(logo_path)
            if not pixmap.isNull():
                self.logo_label.setPixmap(pixmap.scaledToWidth(300, Qt.SmoothTransformation))
            else:
                self.logo_label.setText("STARLANCE TELEMETRY")
                self.logo_label.setStyleSheet("color: white; font-size: 24px; font-weight: bold;")
        else:
            self.logo_label.setText("STARLANCE TELEMETRY")
            self.logo_label.setStyleSheet("color: white; font-size: 24px; font-weight: bold;")
        self.logo_label.setAlignment(Qt.AlignLeft)
        header_layout.addWidget(self.logo_label)
        
        # Recording controls
        recording_layout = QVBoxLayout()
        
        self.record_button = QPushButton("🔴 START RECORDING")
        self.record_button.setFixedSize(200, 50)
        self.record_button.setStyleSheet("""
            QPushButton {
                background-color: #2a2a2a;
                border: 3px solid #ff0000;
                color: #ff0000;
                font-size: 14px;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #3a3a3a;
            }
            QPushButton:pressed {
                background-color: #ff0000;
                color: white;
            }
        """)
        self.record_button.clicked.connect(self.toggle_recording)
        recording_layout.addWidget(self.record_button)
        
        self.recording_status = QLabel("● READY TO RECORD")
        self.recording_status.setStyleSheet("color: #ffff00; font-size: 12px; font-weight: bold;")
        recording_layout.addWidget(self.recording_status)
        
        header_layout.addLayout(recording_layout)
        header_layout.addStretch()
        
        # Connection status light
        self.status_light = QLabel()
        self.status_light.setFixedSize(20, 20)
        self.status_light.setStyleSheet("background-color: red; border-radius: 10px;")
        header_layout.addWidget(self.status_light)
        
        layout.addLayout(header_layout)

        # Event flash overlay
        self.event_overlay = QLabel()
        self.event_overlay.setStyleSheet("color: yellow; font-size: 48px; font-weight: bold;")
        self.event_overlay.setAlignment(Qt.AlignCenter)
        self.event_overlay.setText("")
        layout.addWidget(self.event_overlay)

        # Three video feeds row
        self.cam1, self.cam2, self.cam3 = QLabel(), QLabel(), QLabel()
        for i, cam in enumerate([self.cam1, self.cam2, self.cam3]):
            cam.setMinimumSize(800, 600)
            cam.setAlignment(Qt.AlignCenter)
            cam.setStyleSheet("background-color: #1a1a1a; color: white; border: 2px solid #333;")
            cam.setText(f"Camera {i+1} Feed")
        
        video_row = QHBoxLayout()
        video_row.addWidget(self.cam1)
        video_row.addWidget(self.cam2)
        video_row.addWidget(self.cam3)
        layout.addLayout(video_row)

        # Dials
        self.altitude_dial = DialWidget("Altitude", 30000)
        self.velocity_dial = DialWidget("Velocity", 1000)
        dial_row = QHBoxLayout()
        dial_row.addWidget(self.altitude_dial)
        dial_row.addStretch()
        dial_row.addWidget(self.velocity_dial)
        layout.addLayout(dial_row)

        # Plots
        self.altitude_plot = LivePlotWidget("Altitude (ft)")
        self.velocity_plot = LivePlotWidget("Velocity (m/s)")
        plot_row = QHBoxLayout()
        plot_row.addWidget(self.altitude_plot)
        plot_row.addWidget(self.velocity_plot)
        layout.addLayout(plot_row)

        # Telemetry labels
        self.telemetry_labels = {}
        grid = QGridLayout()
        keys = list(self.telemetry_data.keys())
        for i, key in enumerate(keys):
            label = QLabel(f"{key}: {self.telemetry_data[key]}")
            label.setFont(QFont("Orbitron", 16))
            label.setStyleSheet("color: lime; padding: 5px;")
            self.telemetry_labels[key] = label
            row = i // 4
            col = i % 4
            grid.addWidget(label, row, col)
        layout.addLayout(grid)

    def setupThreads(self):
        self.threads = []
        # Create three video threads
        camera_ports = [1, 3, 5]  # Three cameras
        for i, port in enumerate(camera_ports):
            t = VideoThread(port)
            t.frame_ready.connect(self.update_video_frame)
            t.start()
            self.threads.append(t)
        
        self.teensy_thread = TeensyReader()
        self.teensy_thread.data_received.connect(self.process_teensy_data)
        self.teensy_thread.start()

    def update_video_frame(self, frame, camera_id):
        img = QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        pix = QPixmap.fromImage(img).scaled(800, 600, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        if camera_id == 1:
            self.cam1.setPixmap(pix)
        elif camera_id == 3:
            self.cam2.setPixmap(pix)
        elif camera_id == 5:
            self.cam3.setPixmap(pix)

    
    def process_teensy_data(self, data):
        try:
            tokens = data.split()
            parsed_data = {}
            for i in range(0, len(tokens) - 1, 2):
                key = tokens[i].rstrip(':')
                value = tokens[i + 1]
                parsed_data[key] = value

            mapping = {
                "T+": "T+",
                "ALT": "Altitude",
                "VEL": "Velocity",
                "LAT": "Latitude",
                "LON": "Longitude",
                "OREINT_X": "Orientation X",
                "OREINT_Y": "Orientation Y",
                "EVENT": "Event"
            }

            for k, v in parsed_data.items():
                if k in mapping:
                    mapped_key = mapping[k]
                    if k == "EVENT":
                        self.telemetry_data[mapped_key] = v
                        if v != self.last_event:
                            self.last_event = v
                            self.event_overlay.setText(v)
                            self.event_flash_timer.start(2000)
                    else:
                        self.telemetry_data[mapped_key] = float(v)

        except Exception as e:
            print(f"Failed to parse telemetry: {e}")

    def clear_event_flash(self):
        self.event_overlay.setText("")
        self.event_flash_timer.stop()

    def update_telemetry_display(self):
        self.update_connection_status()

        for key, label in self.telemetry_labels.items():
            value = self.telemetry_data[key]
            if key == "Altitude":
                feet = float(value) * 3.28084
                label.setText(f"Altitude: {feet:.1f} ft")
                self.altitude_plot.update_plot(time.time() - self.start_time, feet)
                self.altitude_dial.set_value(feet)
            elif key == "Velocity":
                label.setText(f"Velocity: {value:.1f} m/s")
                self.velocity_plot.update_plot(time.time() - self.start_time, float(value))
                self.velocity_dial.set_value(float(value))
            elif key == "Latitude":
                label.setText(f"Latitude: {value:.6f}")
            elif key == "Longitude":
                label.setText(f"Longitude: {value:.6f}")
            else:
                label.setText(f"{key}: {value}")

        self.recording_manager.record_telemetry(self.telemetry_data)

    def capture_frame_for_recording(self):
        if self.recording_manager.is_recording:
            pixmap = self.grab()  # Capture the current GUI window
            self.recording_manager.record_frame(pixmap)

    def toggle_recording(self):
        if not self.recording_manager.is_recording:
            success, msg = self.recording_manager.start_recording()
            if success:
                self.record_button.setText("⏹️ STOP RECORDING")
                self.recording_status.setText("● RECORDING...")
                self.recording_status.setStyleSheet("color: #ff0000; font-size: 12px; font-weight: bold;")
            else:
                QMessageBox.warning(self, "Recording", msg)
        else:
            success, msg = self.recording_manager.stop_recording()
            if success:
                self.record_button.setText("🔴 START RECORDING")
                self.recording_status.setText("● READY TO RECORD")
                self.recording_status.setStyleSheet("color: #ffff00; font-size: 12px; font-weight: bold;")
            else:
                QMessageBox.warning(self, "Recording", msg)

    def update_connection_status(self):
        if self.teensy_thread.connected:
            self.status_light.setStyleSheet("background-color: green; border-radius: 10px;")
        else:
            self.status_light.setStyleSheet("background-color: red; border-radius: 10px;")

    def closeEvent(self, event):
        self.cmd_window.close()
        for t in self.threads:
            t.stop()
            t.wait()
        self.teensy_thread.stop()
        self.teensy_thread.wait()
        if self.recording_manager.is_recording:
            self.recording_manager.stop_recording()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    reply, ok = QInputDialog.getItem(None, "Mode Selection", "Choose mode:", ["Real Mode", "Demo Mode"], 1, False)
    if ok:
        if reply == "Demo Mode":
            DEMO_MODE = True
            PORT_NAME = "DEMO"
        else:
            PORT_NAME, ok = QInputDialog.getText(None, "Enter Port", "Enter Teensy COM Port:")
            if not ok:
                sys.exit(0)
    else:
        sys.exit(0)

    window = TelemetryGUI()
    window.showFullScreen()
    sys.exit(app.exec_())
