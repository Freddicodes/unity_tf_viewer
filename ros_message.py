"""
ros_message.py — Deserializes TF messages from ros_tcp_connector.

Supports both serialization formats transparently:

  ROS1 (std_msgs, roscpp style) — no header, plain little-endian binary:
    tf2_msgs/TFMessage:
      uint32 count
      TransformStamped[count]:
        Header:
          uint32 seq
          uint32 stamp_sec
          uint32 stamp_nsec
          string frame_id          # [uint32 len][bytes] — no null terminator
        string child_frame_id
        Transform:
          Vector3 translation      # 3 × float64
          Quaternion rotation      # float64 x, y, z, w

  ROS2 (CDR / OMG-RTPS style) — 4-byte encapsulation header then CDR-encoded
  fields with natural alignment padding:
    Bytes 0-1: 0x00 0x01 (little-endian CDR) or 0x00 0x00 (big-endian CDR)
    Bytes 2-3: options (usually 0x00 0x00)
    Remaining: CDR payload — same logical fields as ROS1 but WITH alignment

Wire framing (identical for both ROS versions, ros_tcp_connector protocol):
    [uint32 LE] topic name length
    [N bytes]   topic name (UTF-8, no null terminator)
    [uint32 LE] message payload length
    [M bytes]   serialized message

Auto-detection:
    payload[0] == 0x00 and payload[1] in {0x00, 0x01}  → ROS2 CDR
    Otherwise                                           → ROS1 plain binary
"""

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Transform:
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)


@dataclass
class TransformStamped:
    sec: int = 0
    nanosec: int = 0          # ROS1 calls this nsec; normalised to nanosec
    frame_id: str = ""
    child_frame_id: str = ""
    transform: Transform = field(default_factory=Transform)
    source: str = "unknown"   # "ros1" | "ros2"

    @property
    def timestamp(self) -> float:
        return self.sec + self.nanosec * 1e-9


# ---------------------------------------------------------------------------
# ROS1 reader — plain little-endian binary, no alignment padding
# ---------------------------------------------------------------------------

class ROS1Reader:
    """Plain little-endian binary reader for ROS1 message serialisation."""

    def __init__(self, data: bytes, offset: int = 0):
        self._d = data
        self._p = offset

    def _remaining(self) -> int:
        return len(self._d) - self._p

    def uint32(self) -> int:
        v = struct.unpack_from("<I", self._d, self._p)[0]
        self._p += 4
        return v

    def int32(self) -> int:
        v = struct.unpack_from("<i", self._d, self._p)[0]
        self._p += 4
        return v

    def float64(self) -> float:
        v = struct.unpack_from("<d", self._d, self._p)[0]
        self._p += 8
        return v

    def string(self) -> str:
        length = self.uint32()
        raw = self._d[self._p: self._p + length]
        self._p += length
        return raw.decode("utf-8", errors="replace")

    def vector3(self) -> Vector3:
        return Vector3(self.float64(), self.float64(), self.float64())

    def quaternion(self) -> Quaternion:
        return Quaternion(self.float64(), self.float64(),
                          self.float64(), self.float64())

    def header(self) -> Tuple[int, int, str]:
        """Reads std_msgs/Header. Returns (sec, nsec, frame_id)."""
        _seq     = self.uint32()   # sequence number — discard
        sec      = self.uint32()
        nsec     = self.uint32()
        frame_id = self.string()
        return sec, nsec, frame_id

    def transform_stamped(self) -> TransformStamped:
        sec, nsec, frame_id = self.header()
        child_frame_id = self.string()
        translation    = self.vector3()
        rotation       = self.quaternion()
        return TransformStamped(
            sec=sec, nanosec=nsec,
            frame_id=frame_id, child_frame_id=child_frame_id,
            transform=Transform(translation, rotation),
            source="ros1",
        )

    def tf_message(self) -> List[TransformStamped]:
        count = self.uint32()
        if count > 1024:
            return []
        out = []
        for _ in range(count):
            out.append(self.transform_stamped())
        return out


# ---------------------------------------------------------------------------
# ROS2 / CDR reader — natural alignment padding after 4-byte header
# ---------------------------------------------------------------------------

class CDRReader:
    """
    Minimal CDR reader for ROS2 messages (OMG CDR 1.0 little-endian).

    Alignment is computed relative to byte 0 of the full payload (i.e. the
    CDR encapsulation header counts toward alignment).  The header occupies
    bytes 0-3; content starts at byte 4.
    """

    def __init__(self, data: bytes, little_endian: bool = True,
                 start_offset: int = 4):
        self._d      = data
        self._p      = start_offset
        self._le     = little_endian
        self._origin = start_offset  # alignment is relative to body start

    def _align(self, size: int):
        if size > 1:
            rem = (self._p - self._origin) % size
            if rem:
                self._p += size - rem

    def uint32(self) -> int:
        self._align(4)
        fmt = "<I" if self._le else ">I"
        v = struct.unpack_from(fmt, self._d, self._p)[0]
        self._p += 4
        return v

    def int32(self) -> int:
        self._align(4)
        fmt = "<i" if self._le else ">i"
        v = struct.unpack_from(fmt, self._d, self._p)[0]
        self._p += 4
        return v

    def float64(self) -> float:
        self._align(8)
        fmt = "<d" if self._le else ">d"
        v = struct.unpack_from(fmt, self._d, self._p)[0]
        self._p += 8
        return v

    def string(self) -> str:
        length = self.uint32()
        if length == 0:
            return ""
        raw = self._d[self._p: self._p + length]
        self._p += length
        # CDR strings are null-terminated; strip it
        return raw.rstrip(b"\x00").decode("utf-8", errors="replace")

    def vector3(self) -> Vector3:
        return Vector3(self.float64(), self.float64(), self.float64())

    def quaternion(self) -> Quaternion:
        return Quaternion(self.float64(), self.float64(),
                          self.float64(), self.float64())

    def transform_stamped(self) -> TransformStamped:
        # ROS2 Header: builtin_interfaces/Time (int32 sec, uint32 nanosec)
        #              then string frame_id
        sec      = self.int32()
        nanosec  = self.uint32()
        frame_id = self.string()
        child_frame_id = self.string()
        translation    = self.vector3()
        rotation       = self.quaternion()
        return TransformStamped(
            sec=sec, nanosec=nanosec,
            frame_id=frame_id, child_frame_id=child_frame_id,
            transform=Transform(translation, rotation),
            source="ros2",
        )

    def tf_message(self) -> List[TransformStamped]:
        count = self.uint32()
        if count > 1024:
            return []
        out = []
        for _ in range(count):
            out.append(self.transform_stamped())
        return out


# ---------------------------------------------------------------------------
# Format detection
# ---------------------------------------------------------------------------

def _detect_format(payload: bytes) -> Tuple[bool, bool]:
    """
    Returns (is_ros2_cdr, little_endian).

    CDR encapsulation header layout:
      byte 0: 0x00  (always)
      byte 1: 0x01 = CDR little-endian  |  0x00 = CDR big-endian
      byte 2: 0x00  (options high byte)
      byte 3: 0x00  (options low byte)
    """
    if len(payload) >= 2 and payload[0] == 0x00 and payload[1] in (0x00, 0x01):
        return True, (payload[1] == 0x01)
    return False, True   # default: ROS1 little-endian


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def parse_tf_payload(payload: bytes) -> List[TransformStamped]:
    """
    Parse a tf2_msgs/TFMessage payload, auto-detecting ROS1 vs ROS2 CDR.
    Falls back to the other format if the primary attempt returns nothing
    or raises an exception.
    Returns a (possibly empty) list of TransformStamped.
    """
    if len(payload) < 4:
        return []

    is_ros2, little_endian = _detect_format(payload)

    def try_ros2():
        return CDRReader(payload, little_endian=little_endian,
                         start_offset=4).tf_message()

    def try_ros1():
        return ROS1Reader(payload, offset=0).tf_message()

    primary   = try_ros2 if is_ros2 else try_ros1
    secondary = try_ros1 if is_ros2 else try_ros2

    try:
        result = primary()
        if result:
            return result
    except Exception:
        pass

    try:
        return secondary()
    except Exception:
        return []


def parse_wire_frame(data: bytes) -> Optional[Tuple[str, bytes]]:
    """
    Parse one ros_tcp_connector wire frame.
    Returns (topic_name, payload_bytes) or None on error.

    Frame layout (same for ROS1 and ROS2 ros_tcp_connector):
      [uint32 LE] topic_name_length
      [N bytes]   topic_name (UTF-8)
      [uint32 LE] message_length
      [M bytes]   serialized message
    """
    if len(data) < 8:
        return None
    try:
        topic_len = struct.unpack_from("<I", data, 0)[0]
        if topic_len > 4096 or 4 + topic_len + 4 > len(data):
            return None
        topic = data[4: 4 + topic_len].decode("utf-8", errors="replace")
        msg_len = struct.unpack_from("<I", data, 4 + topic_len)[0]
        payload_start = 4 + topic_len + 4
        if payload_start + msg_len > len(data):
            return None
        payload = data[payload_start: payload_start + msg_len]
        return topic, payload
    except Exception:
        return None
