# ROS TF Viewer

A standalone macOS viewer for ROS2 TF transforms received via **ros_tcp_connector**.
No ROS installation, no `rclpy` — just Python + PyQt6 + PyOpenGL.

---

## Architecture

```
ROS2 Robot ──── ros_tcp_connector ────TCP────▶ This viewer (TCP server)
                  (TCP client)        port 10000
```

The viewer acts as the **TCP endpoint** (server). `ros_tcp_connector` on your
robot connects to it and streams `/tf` and `/tf_static` messages.

---

## Setup

### 1. Install dependencies

```bash
# macOS (Apple Silicon or Intel)
pip install -r requirements.txt
```

> On Apple Silicon (M1/M2/M3), `PyOpenGL-accelerate` may not have a prebuilt
> wheel. That's fine — pure-Python PyOpenGL still works.

### 2. Run the viewer

```bash
python main.py
```

The viewer listens on **TCP port 10000** by default.

---

## ROS2 Side Configuration

### Install ros_tcp_connector (if not already)

```bash
cd ~/ros2_ws/src
git clone -b main https://github.com/Unity-Technologies/ROS-TCP-Connector.git
cd ..
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

### Launch ros_tcp_endpoint pointing at your Mac

```bash
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args \
  -p ROS_IP:=<YOUR_MAC_IP> \
  -p ROS_TCP_PORT:=10000
```

Replace `<YOUR_MAC_IP>` with the IP of your Mac on the network (e.g. `192.168.1.42`).

### Verify TF is being published

```bash
ros2 topic echo /tf
ros2 topic echo /tf_static
```

If transforms appear there, they'll show up in the viewer automatically.

---

## Viewer Controls

| Input | Action |
|-------|--------|
| Left-drag | Orbit camera |
| Middle-drag | Pan |
| Scroll wheel | Zoom |
| `R` key | Reset camera |
| Click frame in list | Show transform details |

---

## ROS2 Coordinate Convention

| Axis | Color | Direction |
|------|-------|-----------|
| X    | Red   | Forward   |
| Y    | Green | Left      |
| Z    | Blue  | Up        |

---

## Project Structure

```
ros_tf_viewer/
├── main.py           # Entry point
├── tcp_listener.py   # TCP server + ros_tcp_connector protocol parser
├── ros_message.py    # CDR deserializer for TFMessage
├── tf_tree.py        # TF frame graph + transform chaining
├── gl_viewport.py    # PyQt6 + PyOpenGL 3D viewport
├── viewer_window.py  # Main Qt window + UI
└── requirements.txt
```

---

## Troubleshooting

**No frames appear after connecting:**
- Check that ros_tcp_endpoint is pointing at the correct IP/port.
- Confirm `/tf` or `/tf_static` topics have data: `ros2 topic hz /tf`.

**PyOpenGL error on Apple Silicon:**
- The frame list and detail panel still work without OpenGL.
- Install via Homebrew: `brew install pyopengl` may help.

**Port already in use:**
- Change the port in `main.py`: `TFListener(host="0.0.0.0", port=XXXX)`.
- Update ros_tcp_endpoint's `ROS_TCP_PORT` accordingly.
