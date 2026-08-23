# gps2xyz

`gps2xyz` is a ROS 1 node that converts WGS84 GNSS fixes (`sensor_msgs/NavSatFix`)
to a local East-North-Up (ENU) trajectory and publishes it as `nav_msgs/Path`.

Unlike a fixed degrees-per-meter approximation, the converter first maps geodetic
coordinates to WGS84 Earth-Centered, Earth-Fixed (ECEF) coordinates and then rotates
them into a local ENU frame. This keeps the local trajectory metrically consistent
at different latitudes and removes the need to tune latitude/longitude resolution
constants.

## Features

- WGS84 geodetic to ECEF to local ENU conversion
- Automatic origin from the first valid GNSS fix or a configured fixed origin
- GNSS status, coordinate, and optional covariance validation
- Optional altitude suppression for 2D receivers
- Configurable yaw alignment, distance downsampling, and bounded path history
- Correct ROS timestamps and latched RViz path output
- Compatibility aliases for the original parameter names

## Requirements

- Ubuntu with ROS 1 (tested design target: Melodic/Noetic)
- Catkin
- Standard ROS packages: `roscpp`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `rviz`

No external geodesy library is required.

## Installation

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Shidabot/gps2xyz.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Usage

Start the converter and RViz:

```bash
roslaunch gps_to_xyz gps_to_xyz.launch gps_topic:=/gps
rosbag play your_data.bag
```

Run without RViz:

```bash
roslaunch gps_to_xyz gps_to_xyz.launch gps_topic:=/gps open_rviz:=false
```

Use a fixed origin instead of the first valid fix:

```bash
roslaunch gps_to_xyz gps_to_xyz.launch \
  auto_origin:=false \
  origin_latitude:=31.2304 \
  origin_longitude:=121.4737 \
  origin_altitude:=5.0
```

The node subscribes to `/gps` and publishes `/gpsTrack` by default. Both topics can
be changed with launch arguments.

## Parameters

| Parameter | Default | Description |
| --- | ---: | --- |
| `gps_topic` | `/gps` | Input `sensor_msgs/NavSatFix` topic |
| `path_topic` | `/gpsTrack` | Output `nav_msgs/Path` topic |
| `output_frame` | `map` | Frame assigned to the local ENU path |
| `auto_origin` | `true` | Use the first valid fix as the ENU origin |
| `origin_latitude` | `0.0` | Fixed-origin latitude in degrees |
| `origin_longitude` | `0.0` | Fixed-origin longitude in degrees |
| `origin_altitude` | `0.0` | Fixed-origin ellipsoidal altitude in metres |
| `yaw_offset_deg` | `0.0` | Counter-clockwise rotation of ENU XY coordinates |
| `use_altitude` | `true` | Preserve vertical GNSS displacement |
| `min_distance_m` | `0.2` | Minimum 3D distance between retained path points |
| `max_position_variance` | `0.0` | Reject fixes above this covariance diagonal; `0` disables filtering |
| `max_path_points` | `10000` | Maximum retained poses; `0` keeps unlimited history |
| `open_rviz` | `true` | Start RViz from the launch file |

`max_position_variance` is expressed in square metres because `NavSatFix` stores
position covariance as variance. Unknown covariance is not rejected.

## Coordinate convention

Before applying `yaw_offset_deg`, the output axes are:

- `x`: East
- `y`: North
- `z`: Up

The output is a local tangent-plane representation. It is intended for local and
regional trajectories, visualization, and comparison鈥攏ot global navigation across
very large distances.

## License

BSD 3-Clause License. See [LICENSE](LICENSE).

