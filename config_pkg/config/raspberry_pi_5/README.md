# Raspberry Pi 5 live VIO

This folder contains a guarded template and a ready configuration for one IMX708 image stream
and the `sensor_msgs/msg/Imu` stream produced by the PX4 converter. Live VIO does not need a
EuRoC dataset. A recorded calibration sequence is still useful for camera intrinsics,
camera-to-IMU extrinsics, IMU noise, and time-offset calibration.

## Before launching

1. `imx708_px4_vio.yaml` is populated from the supplied 800x600 Kalibr pinhole-radtan-rs result,
   the PX4 IMU calibration, and the calibrated camera-IMU transform/time offset. Its tracker is
   limited to 15 Hz and 120 features for the Pi 5; the camera calibration was collected at 30 Hz.
   Keep the same 800x600 sensor mode at runtime. Use `imx708_template.yaml` for another calibration
   and keep `calibration_required: 1` until all values are filled; both VINS nodes refuse to start
   with the template.
2. Use the same `camera_ros` output resolution, sensor mode, crop, and focus as calibration. The
   supplied camera intrinsics and rolling-shutter timing match 800x600. The VINS feature tracker
   reads its intrinsics from this YAML; it does not automatically copy `/camera/camera_info`.
3. `rolling_shutter_tr` is the full-frame sensor readout duration in seconds. The estimator divides
   it by the configured image height to compute the per-row timing. Its value depends on the active
   IMX708 sensor mode; do not use the frame period as a substitute.
4. `extrinsicRotation` and `extrinsicTranslation` are the transform from camera coordinates into
   the IMU coordinates (`imu^R_cam`, `imu^T_cam`). The supplied PX4 converter rotates FRD data into
   FLU by negating Y and Z, so calibrate the transform for that converted FLU frame. Translation is
   in meters. IMU acceleration and angular velocity must be in m/s^2 and rad/s.
5. Run the camera and PX4 converter before launching VINS. The bridge subscribes to `/imu/data`
   and publishes the converted-clock stream on `/imu/data_vio`, which is the IMU topic VINS uses.
   Update `image_topic` in the YAML if the camera uses a different topic, and use the
   `imu_input_topic` launch argument if the converter uses a different IMU topic.
   `imu_clock_bridge` maps the converter's system-clock stamps to the `CLOCK_BOOTTIME`
   domain used by libcamera's default sensor timestamp, then publishes `/imu/data_vio`.
   Your `run_camera_node.sh` can cap the camera at 15 Hz by passing
   `--ros-args -p FrameDurationLimits:="[66667,66667]"`. This reduces camera and DDS load; keep
   the output at 800x600 and check `ros2 topic hz /camera/image_raw` after restarting the camera.

The bridge expects the PX4 uXRCE-DDS client to synchronize message timestamps with the companion
computer clock. On firmware versions that expose `UXRCE_DDS_SYNCT`, leave it enabled. Verify
`/fmu/out/timesync_status` reports the DDS source and that `/imu/data` stamps are close to the Pi's
current system time. If the bridge reports that the IMU stamp is not close to the system clock,
check PX4/uXRCE synchronization and the converter before proceeding. Do not switch `camera_ros` to
`use_node_time=true` for this configuration: that replaces the sensor acquisition timestamp with
node time and weakens the rolling-shutter timing. Keep `use_sim_time` false for the live Pi setup.

## Build and run

Rebuild the changed packages in the ROS 2 workspace and source its `install/setup.bash`:

```bash
colcon build --packages-select feature_tracker vins_estimator config_pkg
source install/setup.bash
```

Then launch the completed configuration:

```bash
ros2 launch config_pkg pi5_vio.launch.py
```

Pass `config_file:=/absolute/path/to/another.yaml` to select a different configuration.

The launch starts the IMU clock bridge, feature tracker, and VINS estimator. It does not start the
camera or PX4 converter; keep those running separately. The tracker and estimator use sensor-data
QoS for their camera and IMU inputs.

On Ubuntu, `camera_ros` may need to be built against Raspberry Pi's `libcamera` fork for full
Camera Module 3/IMX708 support. The upstream `camera_ros` documentation describes that path:
<https://github.com/christianrauch/camera_ros#build-instructions>.
