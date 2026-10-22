# Raspberry Pi 5 live VIO

This folder contains a guarded starter configuration for one IMX708 image stream and the
`sensor_msgs/msg/Imu` stream produced by the PX4 converter. Live VIO does not need a EuRoC
dataset. A recorded calibration sequence is still useful for camera intrinsics, camera-to-IMU
extrinsics, IMU noise, and time-offset calibration.

## Before launching

1. Copy `imx708_template.yaml` to a writable location and fill in the actual image dimensions,
   intrinsics, camera-to-IMU transform, IMU noise values, and rolling-shutter readout time.
   Keep `calibration_required: 1` until all those values are ready; both VINS nodes refuse to
   start with the template. Then set it to `0`.
2. Calibrate at the same `camera_ros` output resolution, sensor mode, crop, and focus that will be
   used at runtime. The VINS feature tracker reads its own intrinsics from this file; it does not
   automatically copy `/camera/camera_info`.
3. `rolling_shutter_tr` is the full-frame sensor readout duration in seconds. The estimator divides
   it by the configured image height to compute the per-row timing. Its value depends on the active
   IMX708 sensor mode; do not use the frame period as a substitute.
4. `extrinsicRotation` and `extrinsicTranslation` are the transform from camera coordinates into
   the IMU coordinates (`imu^R_cam`, `imu^T_cam`). The supplied PX4 converter rotates FRD data into
   FLU by negating Y and Z, so calibrate the transform for that converted FLU frame. Translation is
   in meters. IMU acceleration and angular velocity must be in m/s^2 and rad/s.
5. Run the camera and PX4 converter before launching VINS. The expected inputs are
   `/camera/image_raw` and `/imu/data`; update `image_topic` in the YAML if the camera uses a
   different topic, and use the `imu_input_topic` launch argument if the converter uses a different
   IMU topic. `imu_clock_bridge` maps the converter's system-clock stamps to the `CLOCK_BOOTTIME`
   domain used by libcamera's default sensor timestamp, then publishes `/imu/data_vio`.

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
ros2 launch config_pkg pi5_vio.launch.py \
  config_file:=/absolute/path/to/imx708.yaml
```

The launch starts the IMU clock bridge, feature tracker, and VINS estimator. It does not start the
camera or PX4 converter; keep those running separately. The tracker and estimator use sensor-data
QoS for their camera and IMU inputs.

On Ubuntu, `camera_ros` may need to be built against Raspberry Pi's `libcamera` fork for full
Camera Module 3/IMX708 support. The upstream `camera_ros` documentation describes that path:
<https://github.com/christianrauch/camera_ros#build-instructions>.
