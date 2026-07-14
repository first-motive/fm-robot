# fm_sensors

Multi-sensor capture layer — the slot where external sensors (cameras, depth, IMUs,
contact) feed the capture pipeline.

## Nodes

### `camera_node` — camera publisher

Opens a camera (a phone-over-IP stream today, wrist cameras later) and republishes the
RAW frames as the standard ROS camera contract, so the vision hand-tracker and the dataset
recorder consume one clean feed — no burned-in overlay, no double compression.

```bash
ros2 run fm_sensors camera_node --ros-args \
  -p camera_source:=http://<phone-ip>:8081/video \
  -p frame_id:=head_cam \
  -p image_topic:=head_cam/image_raw \
  -p info_topic:=head_cam/camera_info
```

Publishes `sensor_msgs/Image` (bgr8) + `sensor_msgs/CameraInfo`. `camera_source` is an int
device index (Linux `/dev/video*`) or an http/rtsp URL. A background reader keeps only the
latest frame and reconnects on stall. Run one instance per camera (head, then each wrist).
The vision mirror session wires this in automatically with `camera_input:=topic`.

`camera_info` is an uncalibrated placeholder — run `camera_calibration` and fill in the
intrinsics for metric/3D use.

### `sensor_node` — placeholder

Generic stub for future sensors (depth, IMU, tactile/contact). Logs that it is up.

## Build Type

`ament_python`.
