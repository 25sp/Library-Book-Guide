# Library-Book-Guide

## Introduction

This project involves designing an car which can establish the map of the library and autonomous navigate
itself to destinations. It works based on laser and Imu.

## Prerequisite

Several binary `ROS` packages are need to install. Please use the following command.

```bash
sudo apt-get install ros-melodic-navigation ros-melodic-serial ros-melodic-usb-cam
```

## Laser SLAM

SLAM is based on RPLIDAR A3. When performing SLAM, ensure all related nodes are activated.

- **Hector SLAM**: Follow these steps to configure and run Laser SLAM:

```bash
cd ~/Library-Book-Guide/workspace
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
sudo chmod 666 /dev/laser
roslaunch rplidar_ros rplidar_a3.launch
```

- **Cartographer**: For better performance, Cartographer is used. Three terminal windows are required:

```bash
# Terminal 1
cd ~/Library-Book-Guide/workspace
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
sudo chmod 666 /dev/laser
roslaunch rplidar_ros rplidar_a3.launch

# Terminal 2
cd ~/Library-Book-Guide/cartographer_workspace
./carto_slam.sh

# Terminal 3
cd ~/Library-Book-Guide/cartographer_workspace
./map_save.sh

# Once the map is successfully saved, quit all terminals.
```

## Active SLAM

To improve mapping efficiency, a simple exploration algorithm is implemented. Follow these steps:

```bash
sudo chmod 666 /dev/arduino
roslaunch explore explore.launch
roslaunch map map_save.launch
```

## Odometry

The package `rf2o_laser_odometry` is primarily used for odometry. Execute the following command to run it:

```bash
roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch
```

## Inertial Measurement Unit (IMU)

The package `imu_tools/imu_complimentary_filter` is primarily used for filtering raw IMU data. Execute the
following command to launch the IMU node and filter the raw IMU data:

```bash
roslaunch imu imu_raw_data.launch
roslaunch imu_complimentary_filter complimentary_filter.launch
```

To set up for IMU in cartographer, make sure the IMU is used in .lua file:

```bash
TRAJECTORY_BUILDER_2D.use_imu_data = true
```

## Localization

### AMCL Localization

- The AMCL algorithm is applied for robot localization. Use the following command:

```bash
roslaunch point_to_point_nav amcl_test.launch
```

### Lidar-Based Localization

- Alternatively, a laser lidar-based localization method is provided. After mapping the target area, run the following command:

```bash
roslaunch point_to_point_nav point_to_point_nav.launch
```


## Keyboard Control

The chassis is controlled by an `ESP32-S3` board. To control movement using a keyboard, follow these steps:

1. Burn `controlled_move.ino` into the `ESP32-S3` board.
2. Open the serial monitor and set the baud rate to 115200.
3. Use the following keys to control movement:
   - `w`: Move forward
   - `a`: Turn left
   - `s`: Move backward
   - `d`: Turn right
   - `x`: Stop

## Navigation

A customized local planner is applied for the navigation stack. Follow these steps:

```bash
roslaunch point_to_point_nav point_to_point_nav.launch

# Use 2D pose estimate to calibrate the initial position.

# Use 2D Nav Goal to set the destination.
```