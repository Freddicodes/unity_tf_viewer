"""
tcp_listener.py — TCP server that accepts ros_tcp_connector connections
and emits parsed TF transforms via callbacks.

ros_tcp_connector acts as the TCP *client*; this viewer acts as the server
(matching the behavior of the ROS TCP Endpoint node).

Handles:
  - ROS1 and ROS2 serialization (auto-detected per message)
  - /tf and /tf_static topics (static transforms are kept indefinitely)
  - The ros_tcp_connector subscriber-registration handshake message
    (topic = "__topic_registration__" or similar; silently ignored)
  - Multiple simultaneous clients
"""

import socket
import struct
import threading
from typing import Callable, Dict, List, Optional
from unity_tf_listener.ros_message import parse_wire_frame, parse_tf_payload, TransformStamped


# Topics that carry tf2_msgs/TFMessage
_TF_TOPIC_SUBSTRINGS = ("tf", "TF")


def _is_tf_topic(topic: str) -> bool:
    return any(s in topic for s in _TF_TOPIC_SUBSTRINGS)


def _is_static_topic(topic: str) -> bool:
    return "static" in topic.lower()


class TFListener:
    """
    TCP server that accepts ros_tcp_connector clients and parses TF messages.

    Usage:
        listener = TFListener(port=10000)
        listener.on_transform_update(my_callback)
        listener.on_status_change(log_callback)
        threading.Thread(target=listener.start, daemon=True).start()
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 10000):
        self.host = host
        self.port = port
        self._running = False
        self._lock = threading.Lock()

        # parent_frame_id -> child_frame_id -> TransformStamped
        self._dynamic: Dict[str, Dict[str, TransformStamped]] = {}
        self._static:  Dict[str, Dict[str, TransformStamped]] = {}

        self._update_callbacks: List[Callable] = []
        self._status_callbacks: List[Callable[[str], None]] = []

        # Diagnostics
        self._msg_count = 0
        self._ros1_count = 0
        self._ros2_count = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def on_transform_update(self, cb: Callable):
        """Register callback(transforms: dict) — called on every TF batch."""
        self._update_callbacks.append(cb)

    def on_status_change(self, cb: Callable[[str], None]):
        """Register callback(message: str) for log/status strings."""
        self._status_callbacks.append(cb)

    def get_transforms(self) -> Dict[str, Dict[str, TransformStamped]]:
        """Snapshot of all known transforms (static + dynamic merged)."""
        with self._lock:
            merged: Dict[str, Dict[str, TransformStamped]] = {}
            for parent, children in self._static.items():
                merged.setdefault(parent, {}).update(children)
            for parent, children in self._dynamic.items():
                merged.setdefault(parent, {}).update(children)
            return merged

    def start(self):
        """Blocking — runs the TCP server. Call from a daemon thread."""
        self._running = True
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self.host, self.port))
        except OSError as e:
            self._status(f"❌ Cannot bind to {self.host}:{self.port} — {e}")
            return

        srv.listen(5)
        srv.settimeout(1.0)
        self._status(f"🟢 Listening on {self.host}:{self.port}")

        while self._running:
            try:
                conn, addr = srv.accept()
                self._status(f"🔗 Client connected: {addr[0]}:{addr[1]}")
                threading.Thread(
                    target=self._handle_client,
                    args=(conn, addr),
                    daemon=True,
                ).start()
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self._status(f"⚠️  Accept error: {e}")

        srv.close()

    def stop(self):
        self._running = False

    # ------------------------------------------------------------------
    # Internal — connection handling
    # ------------------------------------------------------------------

    def _handle_client(self, conn: socket.socket, addr):
        conn.settimeout(5.0)
        buf = b""
        seen_topics: set = set()

        try:
            while self._running:
                try:
                    chunk = conn.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk

                # Drain all complete frames from the buffer
                while True:
                    consumed = self._try_consume_frame(buf, seen_topics)
                    if consumed is None:
                        break          # need more data
                    if consumed == 0:
                        # Bad frame start — advance one byte and retry
                        buf = buf[1:]
                    else:
                        buf = buf[consumed:]

        except Exception as e:
            self._status(f"⚠️  Client error ({addr[0]}): {e}")
        finally:
            conn.close()
            self._status(
                f"🔴 Client disconnected: {addr[0]}:{addr[1]}  "
                f"(msgs: {self._msg_count}, "
                f"ROS1: {self._ros1_count}, ROS2: {self._ros2_count})"
            )

    def _try_consume_frame(self, buf: bytes,
                           seen_topics: set) -> Optional[int]:
        """
        Try to parse and consume one complete wire frame from the start of buf.

        Returns:
          int > 0  — number of bytes consumed (frame was valid)
          0        — frame header looks invalid (caller should skip 1 byte)
          None     — not enough data yet (caller should wait for more)
        """
        if len(buf) < 8:
            return None

        topic_len = struct.unpack_from("<I", buf, 0)[0]

        # Sanity check on topic length
        if topic_len > 4096:
            return 0

        header_end = 4 + topic_len
        if len(buf) < header_end + 4:
            return None

        msg_len   = struct.unpack_from("<I", buf, header_end)[0]
        frame_end = header_end + 4 + msg_len

        # Sanity check on message length (16 MB cap)
        if msg_len > 16 * 1024 * 1024:
            return 0

        if len(buf) < frame_end:
            return None

        frame = buf[:frame_end]
        self._process_frame(frame, seen_topics)
        return frame_end

    # ------------------------------------------------------------------
    # Internal — message processing
    # ------------------------------------------------------------------

    def _process_frame(self, data: bytes, seen_topics: set):
        result = parse_wire_frame(data)
        if result is None:
            return

        topic, payload = result

        # Log new topics once per connection
        if topic not in seen_topics:
            seen_topics.add(topic)
            self._status(f"📡 New topic: {topic!r}  ({len(payload)} B payload)")

        if not _is_tf_topic(topic):
            return

        transforms = parse_tf_payload(payload)
        if not transforms:
            return

        # Update format counters
        self._msg_count += 1
        if transforms[0].source == "ros1":
            self._ros1_count += 1
        else:
            self._ros2_count += 1

        is_static = _is_static_topic(topic)
        store = self._static if is_static else self._dynamic

        with self._lock:
            for tf in transforms:
                store.setdefault(tf.frame_id, {})[tf.child_frame_id] = tf

        self._emit_update()

    # ------------------------------------------------------------------
    # Internal — callbacks
    # ------------------------------------------------------------------

    # noinspection PyBroadException
    def _emit_update(self):
        snapshot = self.get_transforms()
        for cb in self._update_callbacks:
            try:
                cb(snapshot)
            except Exception:
                pass

    # noinspection PyBroadException
    def _status(self, msg: str):
        for cb in self._status_callbacks:
            try:
                cb(msg)
            except Exception:
                pass
