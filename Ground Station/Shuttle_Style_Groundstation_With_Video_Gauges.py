import sys
import serial
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QGridLayout
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import Qt, QTimer

# Serial Port Configuration
SERIAL_PORT = '/dev/tty.usbmodem108845801'  # Update with correct Teensy port
BAUD_RATE = 115200  # Match Teensy's baud rate

# Paths to logo images
SLURPL_LOGO_PATH = "SLURPL.png"  # SLURPL logo

class TelemetryGUI(QWidget):
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rocket Telemetry System")
        self.showFullScreen()  # Full-Screen Mode
        self.setStyleSheet("background-color: black;")  # Set background to black

        # Default telemetry values
        self.telemetry_data = {"T+": 0, "ALT": 0, "VEL": 0, "LAT": 0.0, "LON": 0.0, "ORIENT_X": 0, "ORIENT_Y": 0}

        self.altitude_data, self.velocity_data = [0], [0]

        self.initUI()

        # Serial Connection
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to serial port: {SERIAL_PORT}")  # Debug: Confirm serial connection
        except Exception as e:
            print(f"Warning: Serial port not found. Running in No-Data mode. Error: {e}")  # Debug: Print error
            self.serial_conn = None

        # Video Capture
        self.cap = cv2.VideoCapture(0)

        # Start Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)
    
    def read_telemetry(self):
        if self.serial_conn and self.serial_conn.in_waiting > 0:
            try:
                data = self.serial_conn.readline().decode().strip()
                print(f"Raw data from serial: {data}")  # Debug: Print raw data
                return self.parse_telemetry(data) if data else self.telemetry_data
            except Exception as e:
                print(f"Error reading telemetry: {e}")  # Debug: Print errors
                return self.telemetry_data
        else:
            print("No data available from serial port.")  # Debug: No data
        return self.telemetry_data

    def parse_telemetry(self, data):
        try:
            # Remove the "||RFD||" prefix if present
            if data.startswith("||RFD||"):
                data = data.replace("||RFD||", "").strip()
            
            telemetry_dict = self.telemetry_data.copy()
            print(f"Parsing data: {data}")  # Debug: Print data being parsed
            # Split the data into key-value pairs
            parts = data.split(",")
            for part in parts:
                if ":" in part:
                    key, value = part.split(":")
                    key = key.strip()
                    value = value.strip()
                    print(f"Key: {key}, Value: {value}")  # Debug: Print key-value pairs
                    # Map the keys to the expected format
                    if key == "T+":
                        telemetry_dict["T+"] = float(value)
                    elif key == "ALT":
                        telemetry_dict["ALT"] = float(value)
                    elif key == "VEL":
                        telemetry_dict["VEL"] = float(value)
                    elif key == "LAT":
                        telemetry_dict["LAT"] = float(value)
                    elif key == "LON":
                        telemetry_dict["LON"] = float(value)
                    elif key == "ORIENT_X":
                        telemetry_dict["ORIENT_X"] = float(value)
                    elif key == "ORIENT_Y":
                        telemetry_dict["ORIENT_Y"] = float(value)
            return telemetry_dict
        except Exception as e:
            print(f"Error parsing telemetry: {e}")  # Debug: Print parsing errors
            return self.telemetry_data

    def initUI(self):
        main_layout = QVBoxLayout()

        # Top Layout (Video + SLURPL Logo)
        top_layout = QHBoxLayout()

        # Video in the Middle
        self.video_label = QLabel(self)
        self.video_label.setFixedSize(800, 450)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: black;")
        top_layout.addWidget(self.video_label, alignment=Qt.AlignmentFlag.AlignCenter)

        # SLURPL Logo on the Right
        self.slurpl_logo_label = QLabel(self)
        self.slurpl_logo_label.setFixedSize(300, 150)
        self.slurpl_logo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.slurpl_logo_label.setStyleSheet("background-color: black;")

        # Load and display the SLURPL logo
        slurpl_logo_pixmap = QPixmap(SLURPL_LOGO_PATH)
        if not slurpl_logo_pixmap.isNull():
            scaled_slurpl_logo = slurpl_logo_pixmap.scaled(300, 150, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.slurpl_logo_label.setPixmap(scaled_slurpl_logo)

        top_layout.addWidget(self.slurpl_logo_label, alignment=Qt.AlignmentFlag.AlignRight)

        main_layout.addLayout(top_layout)

        # Telemetry & Graph Layout
        bottom_layout = QHBoxLayout()

        # Telemetry Display (HUD Style)
        telemetry_layout = QGridLayout()
        self.telemetry_labels = {}
        telemetry_keys = ["T+", "ALT", "VEL", "LAT", "LON", "ORIENT_X", "ORIENT_Y"]
        for i, key in enumerate(telemetry_keys):
            label = QLabel(f"{key}: {self.telemetry_data[key]}")
            label.setFont(QFont("Courier", 18, QFont.Bold))
            label.setStyleSheet("color: lime; background-color: black;")
            telemetry_layout.addWidget(label, i // 2, i % 2)
            self.telemetry_labels[key] = label

        bottom_layout.addLayout(telemetry_layout)

        # Graph (Velocity vs Altitude) - Fully Black
        self.figure, self.ax = plt.subplots(figsize=(5, 3))
        self.figure.patch.set_facecolor('black')
        self.ax.set_facecolor("black")
        self.ax.spines['bottom'].set_color('white')
        self.ax.spines['top'].set_color('white')
        self.ax.spines['left'].set_color('white')
        self.ax.spines['right'].set_color('white')
        self.ax.set_title("Velocity vs Altitude", color="white")
        self.ax.set_xlabel("Altitude (m)", color="white")
        self.ax.set_ylabel("Velocity (m/s)", color="white")
        self.ax.tick_params(colors="white")
        self.canvas = FigureCanvas(self.figure)
        bottom_layout.addWidget(self.canvas)

        main_layout.addLayout(bottom_layout)
        self.setLayout(main_layout)

    def update_frame(self):
        print("Updating frame...")  # Debug: Confirm method is being called
        if self.cap is None:
            return

        ret, frame = self.cap.read()
        if ret:
            self.telemetry_data = self.read_telemetry()

            # Update telemetry labels
            for key, label in self.telemetry_labels.items():
                label.setText(f"{key}: {self.telemetry_data[key]}")  # Update labels in real-time

            # Convert frame and display video
            frame_resized = cv2.resize(frame, (800, 450))
            rgb_image = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            height, width, channel = rgb_image.shape
            qimage = QImage(rgb_image.data, width, height, channel * width, QImage.Format.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qimage))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TelemetryGUI()
    window.show()
    sys.exit(app.exec())