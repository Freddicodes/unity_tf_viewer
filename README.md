# ROS TF Viewer

A standalone macOS viewer for ROS2 TF transforms received via [ros_tcp_connector](https://github.com/Unity-Technologies/ROS-TCP-Connector).
No ROS installation, no `rclpy` — just Python + PyQt6 + PyOpenGL.

---

## Architecture

```text
ROS2 Robot ──── ros_tcp_connector ────TCP────▶ This viewer (TCP server)
                  (TCP client)        port 10000
```

The viewer acts as the **TCP endpoint** (server). `ros_tcp_connector` on your
robot connects to it and streams `/tf` and `/tf_static` messages.

---

## Setup

### 1. Install dependencies

```bash
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

## Viewer Controls

| Input | Action |
| ----- | ------ |
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
