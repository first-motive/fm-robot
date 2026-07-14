"""camera_node — publish a camera stream as a standard ROS image topic.

This is the fm_sensors capture slot made real: one node opens a camera (a phone-over-IP
stream today, wrist cameras later as more instances) and republishes the RAW frames so the
vision hand-tracker AND the dataset recorder consume ONE clean feed — no burned-in skeleton
overlay, no double compression. Publishing image_raw + camera_info is the standard ROS
camera contract, so any consumer (tracker, recorder, rviz, image_pipeline) just works.

Publishes:
    <image_topic>  sensor_msgs/Image       bgr8, at the stream rate (default head_cam/image_raw)
    <info_topic>   sensor_msgs/CameraInfo  width/height (+ intrinsics once calibrated)

Source: an int device index (Linux /dev/video*) or an http/rtsp URL (a phone IP-webcam on
the OrbStack network — same convention as fm_teleop_vision). A background reader thread keeps
only the latest frame and reconnects on stall, so a slow subscriber never lags the stream.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameter("camera_source", "0")            # int index or http/rtsp URL
        self.declare_parameter("frame_id", "head_cam")
        self.declare_parameter("image_topic", "head_cam/image_raw")
        self.declare_parameter("info_topic", "head_cam/camera_info")
        self.declare_parameter("publish_rate", 30.0)            # cap; publishes the latest frame
        self.declare_parameter("reconnect_backoff_s", 1.0)

        gp = self.get_parameter
        self._frame_id = gp("frame_id").value
        self._backoff = float(gp("reconnect_backoff_s").value)
        self._source = self._resolve_source(gp("camera_source").value)

        self._img_pub = self.create_publisher(
            Image, gp("image_topic").value, qos_profile_sensor_data)
        self._info_pub = self.create_publisher(
            CameraInfo, gp("info_topic").value, qos_profile_sensor_data)

        self._lock = threading.Lock()
        self._frame = None                                      # latest bgr numpy frame
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._run, name="camera_reader", daemon=True)
        self._reader.start()
        period = 1.0 / max(float(gp("publish_rate").value), 1.0)
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info("camera_node up (source=%r, frame=%s)"
                               % (self._source, self._frame_id))

    @staticmethod
    def _resolve_source(val):
        s = str(val)
        try:
            return int(s)
        except ValueError:
            return s

    # --- background reader: blocking capture + reconnect (keeps only the newest frame) ---
    def _run(self):
        import cv2  # deferred so the node imports without opencv (for unit tests)
        cap = None
        while not self._stop.is_set():
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(self._source)
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)         # bound latency to ~1 frame
                except Exception:
                    pass
                if not cap.isOpened():
                    self.get_logger().warn(
                        "camera open failed (%r); retrying" % self._source,
                        throttle_duration_sec=5.0)
                    cap = None
                    time.sleep(self._backoff)
                    continue
            ok, frame = cap.read()
            if not ok or frame is None:
                self.get_logger().warn("camera read failed; reconnecting",
                                       throttle_duration_sec=5.0)
                cap.release()
                cap = None
                time.sleep(self._backoff)
                continue
            with self._lock:
                self._frame = frame
        if cap is not None:
            cap.release()

    # --- timer: publish the latest frame + matching camera_info ---
    def _publish(self):
        with self._lock:
            frame = self._frame
        if frame is None:
            return
        h, w = frame.shape[:2]
        stamp = self.get_clock().now().to_msg()

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self._frame_id
        img.height = h
        img.width = w
        img.encoding = "bgr8"
        img.is_bigendian = 0
        img.step = w * 3
        img.data = np.ascontiguousarray(frame).tobytes()
        self._img_pub.publish(img)

        info = CameraInfo()
        info.header = img.header
        info.width = w
        info.height = h
        # Uncalibrated placeholder so consumers have the intrinsics slot — swap in a real
        # calibration (camera_calibration) for metric/3D use.
        info.distortion_model = "plumb_bob"
        self._info_pub.publish(info)

    def destroy_node(self):
        self._stop.set()
        if self._reader.is_alive():
            self._reader.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
