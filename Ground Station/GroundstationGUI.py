#This code was developed by Ryan Santiago for the IREC 2025 Competition,the required libraries can be installed using pip install pyqt5 opencv-python pyserial pygrabber


import sys
import os
import cv2
import threading
import time
import serial
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtWidgets import (
    QApplication, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QSizePolicy,
    QComboBox, QPushButton, QGridLayout, QFrame,
)
from PyQt5.QtGui import (
    QImage, QPixmap, QFont, QPainter, QColor, QBrush, QConicalGradient
)
from PyQt5.QtCore import Qt, QTimer, QRect, pyqtSignal, QThread

port_name = input("Enter Teensy Port, If you don't know run Detect.py: ")

def find_camera_by_name(partial_name):
    try:
        from pygrabber.dshow_graph import FilterGraph
        graph = FilterGraph()
        devices = graph.get_input_devices()
        for i, name in enumerate(devices):
            if "z fold" in name.lower():
                continue  # skip the phone camera
            print(f"{i}: {name}")
            if partial_name.lower() in name.lower():
                print(f"Selected device for '{partial_name}' at index {i}: {name}")
                return i
        print(f"Device with name containing '{partial_name}' not found. Defaulting to index 0.")
        return 0
    except Exception as e:
        print(f"Camera detection failed: {e}")
        return 0
    except Exception as e:
        print(f"Camera detection failed: {e}")
        return 0
    except Exception as e:
        print(f"Camera detection failed: {e}")
        return 0

def find_analog_capture_device():
    try:
        from pygrabber.dshow_graph import FilterGraph
        graph = FilterGraph()
        devices = graph.get_input_devices()
        for i, name in enumerate(devices):
            print(f"{i}: {name}")
            if "av to usb2.0" in name.lower():
                print(f"Analog input found at index {i}: {name}")
                return i
        print("Analog input not found. Defaulting to index 0.")
        return 0
    except Exception as e:
        print(f"Analog detection failed: {e}")
        return 0
    except Exception as e:
        print(f"Camera detection failed: {e}")
        return 0
    except Exception as e:
        print(f"Camera detection failed: {e}")
        return 0


class TeensyReader(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, port=port_name, baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.connected = False
        self.command = '0'

    def run(self):
        while self.running:
            try:
                if not self.connected:
                    self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
                    self.connected = True
                    print(f"Connected to Teensy on {self.port}")

                if self.connected:
                    try:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        if line:
                            print(f"Raw line: {line}")
                            self.data_received.emit(line)
                        if self.command != '0':
                            self.serial_conn.write(self.command)
                            self.command = '0'
                    except Exception as e:
                        print(f"Lost connection or read error: {e}")
                        self.connected = False
                        try:
                            self.serial_conn.close()
                        except:
                            pass
                        time.sleep(1)
            except Exception as e:
                print(f"Teensy connection error: {e}")
                self.connected = False
                time.sleep(1)

    def stop(self):
        self.running = False
        if self.connected:
            try:
                self.serial_conn.close()
            except:
                pass
            self.connected = False


class VideoThread(QThread):
    frame_ready = pyqtSignal(object, int)

    def __init__(self, camera_id, source_type="webcam"):
        super().__init__()
        self.camera_id = camera_id
        self.source_type = source_type
        self.running = True

    def run(self):
        cap = cv2.VideoCapture(self.camera_id)
        while self.running:
            ret, frame = cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.frame_ready.emit(frame, self.camera_id)
            time.sleep(0.03)
        cap.release()

    def stop(self):
        self.running = False


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
        side = min(self.width(), self.height())
        rect = QRect(10, 10, side - 20, side - 20)
        painter.setPen(QColor(255, 255, 255))
        painter.setBrush(QColor(0, 0, 0, 180))
        painter.drawEllipse(rect)
        gradient = QConicalGradient(rect.center(), 90)
        gradient.setColorAt(0, QColor(0, 255, 0))
        gradient.setColorAt(0.5, QColor(255, 255, 0))
        gradient.setColorAt(1, QColor(255, 0, 0))
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(gradient))
        arc_angle = int(-self.value / self.max_value * 180 * 16)
        painter.drawPie(rect, 90 * 16, arc_angle)
        painter.setPen(QColor(255, 255, 255))
        painter.setFont(QFont("Monospace", 16, QFont.Bold))
        painter.drawText(rect, Qt.AlignCenter, f"{int(self.value)}")
        painter.setFont(QFont("Monospace", 12, QFont.Bold))
        painter.drawText(rect, Qt.AlignBottom | Qt.AlignHCenter, self.title)


class TelemetryGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rocket Telemetry GUI")
        self.setStyleSheet("background-color: black;")
        self.setGeometry(100, 100, 1280, 720)
        self.telemetry_data = {
            "T+": 0, "Altitude": 0, "Velocity": 0,
            "Latitude": 0.0, "Longitude": 0.0,
            "Orientation X": 0, "Orientation Y": 0
        }
        self.sim_time = 0.0
        self.analog_camera_id = find_camera_by_name("AV TO USB2.0")
        self.digital_camera_id = find_camera_by_name("OBS Virtual Camera")
        self.teensy_port = port_name
        self.initUI()
        self.setupCameraThreads()
        self.setupTeensyReader()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_telemetry_display)
        self.timer.start(100)

    def keyPressEvent(self, event: QKeyEvent):
        self.teensy_thread.command = (event.text()).encode()

    def initUI(self):
        main_layout = QVBoxLayout(self)
        self.setLayout(main_layout)

        height = self.height()
        vid_height = int(height/1.5)

        self.analog_video_frame = QLabel("Analog Video")
        self.analog_video_frame.setAlignment(Qt.AlignCenter)
        self.analog_video_frame.setMinimumSize(int(4*vid_height/3), vid_height)
        self.analog_video_frame.setMaximumSize(int(4*vid_height/3), vid_height)
        self.digital_video_frame = QLabel("Digital Video")
        self.digital_video_frame.setAlignment(Qt.AlignCenter)
        self.digital_video_frame.setMinimumSize(int(16*vid_height/9), vid_height)
        self.digital_video_frame.setMaximumSize(int(16*vid_height/9), vid_height)

        video_layout = QHBoxLayout()
        video_layout.addWidget(self.analog_video_frame)
        video_layout.addWidget(self.digital_video_frame)

        main_layout.addLayout(video_layout)

        self.altitude_dial = DialWidget("Altitude", 10000)
        self.velocity_dial = DialWidget("Velocity", 1000)

        dial_layout = QHBoxLayout()
        dial_layout.addWidget(self.altitude_dial)
        dial_layout.addStretch()
        dial_layout.addWidget(self.velocity_dial)
        main_layout.addLayout(dial_layout)

        telemetry_container = QWidget()
        telemetry_layout = QHBoxLayout(telemetry_container)
        self.telemetry_labels = {}
        for key in self.telemetry_data:
            label = QLabel(f"{key}: {self.telemetry_data[key]}")
            label.setStyleSheet("color: lime;")
            self.telemetry_labels[key] = label
            telemetry_layout.addWidget(label)

        main_layout.addWidget(telemetry_container)

        self.status_label = QLabel("Status: Ready")
        self.status_label.setStyleSheet("color: white;")
        main_layout.addWidget(self.status_label)

    def setupCameraThreads(self):
        self.analog_video_thread = VideoThread(self.analog_camera_id, "analog")
        self.analog_video_thread.frame_ready.connect(self.update_video_frame)
        self.analog_video_thread.start()
        self.digital_video_thread = VideoThread(self.digital_camera_id, "digital")
        self.digital_video_thread.frame_ready.connect(self.update_video_frame)
        self.digital_video_thread.start()

    def setupTeensyReader(self):
        self.teensy_thread = TeensyReader(self.teensy_port)
        self.teensy_thread.data_received.connect(self.process_teensy_data)
        self.teensy_thread.start()

    def update_video_frame(self, frame, camera_id):
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        target = self.analog_video_frame if camera_id == self.analog_camera_id else self.digital_video_frame
        target.setPixmap(pixmap.scaled(target.width(), target.height()))

    def process_teensy_data(self, line):
        try:
            data_dict = {}
            tokens = line.split()
            for i in range(0, len(tokens) - 1, 2):
                key = tokens[i].replace(":", "").strip()
                value = tokens[i + 1].strip()
                data_dict[key] = value

            key_mapping = {
                "ALT": "Altitude",
                "VEL": "Velocity",
                "LAT": "Latitude",
                "LON": "Longitude",
                "OREINT_X": "Orientation X",
                "OREINT_Y": "Orientation Y",
                "T+": "T+"
            }

            for key, value in data_dict.items():
                if key in key_mapping:
                    display_key = key_mapping[key]
                    try:
                        self.telemetry_data[display_key] = float(value)
                    except ValueError:
                        self.telemetry_data[display_key] = value

            self.status_label.setText(f"Status: Connected - Last data: {time.strftime('%H:%M:%S')}")
        except Exception as e:
            print(f"Failed to parse Teensy line: {line} -- Error: {e}")

    def update_telemetry_display(self):
        for key, label in self.telemetry_labels.items():
            label.setText(f"{key}: {self.telemetry_data[key]}")
        if isinstance(self.telemetry_data["Altitude"], (int, float)):
            self.altitude_dial.set_value(self.telemetry_data["Altitude"])
        if isinstance(self.telemetry_data["Velocity"], (int, float)):
            self.velocity_dial.set_value(self.telemetry_data["Velocity"])

    def closeEvent(self, event):
        self.analog_video_thread.stop()
        self.digital_video_thread.stop()
        self.teensy_thread.stop()
        self.analog_video_thread.wait()
        self.digital_video_thread.wait()
        self.teensy_thread.wait()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TelemetryGUI()
    window.showFullScreen()
    sys.exit(app.exec_())
