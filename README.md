# contactile_ros

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/contactile_ros
$ cd ~/ros2_ws
$ colcon build
```

## Usage

```shell
$ ros2 launch contactile_bringup contactile.launch.py
```

```shell
$ ros2 topic echo /contactile/sensor_0
```

<p align="center">
  <img src="./docs/demo.gif" width="100%" />
</p>