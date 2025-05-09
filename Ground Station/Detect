#This code is used to detect camera devices and the correct port of the teensy, to help with easier setup. 

import cv2
import serial.tools.list_ports
from pygrabber.dshow_graph import FilterGraph

def list_video_devices():
    print("=== Video Devices ===")
    try:
        graph = FilterGraph()
        devices = graph.get_input_devices()
        for i, name in enumerate(devices):
            print(f"{i}: {name}")
    except Exception as e:
        print(f"Failed to list video devices: {e}")

def list_serial_ports():
    print("\n=== Serial Ports (COM) ===")
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No COM ports detected.")
    for port in ports:
        print(f"{port.device} - {port.description}")

if __name__ == "__main__":
    list_video_devices()
    list_serial_ports()
