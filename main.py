#!/usr/bin/env python3
"""
ROS TF Viewer — listens to ros_tcp_connector TF messages and displays them
in a ROS2 coordinate system (X=forward/red, Y=left/green, Z=up/blue).
No rclpy or ROS installation required.
"""

import sys
import threading
from PyQt6.QtWidgets import QApplication
from unity_tf_listener.tcp_listener import TFListener
from unity_tf_listener.viewer_window import ViewerWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("ROS TF Viewer")

    listener = TFListener(host="0.0.0.0", port=10000)
    window = ViewerWindow(listener)
    window.show()

    # Start TCP listener in background thread
    listener_thread = threading.Thread(target=listener.start, daemon=True)
    listener_thread.start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
