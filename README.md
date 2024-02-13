# ROS2 Mapping with RPLIDAR A1 and Pygame Visualization

ROS2 package tailored for environment mapping using the RPLIDAR A1 sensor, along with real-time visualization powered by pygame. With a Docker environment provided, users can effortlessly deploy and utilize the mapping capabilities without worrying about setup complexities.

## Clone the package in ros environment

**Step 1** Source the setup files

```bash
source /opt/ros/iron/setup.bash
```

**Step 2** Navigate to ros workspace source directory

```bash
cd home/ros/ros2_ws/src
```

**Step 3** Clone ros2_rplidar_sub package from Github

Ensure you're still in the ros2_ws/src directory before you clone:

```bash
git clone https://github.com/dhanushshettigar/ros2_rplidar_sub.git
```

**Step 4** Build ros2_rplidar_sub package

From the root of your workspace (ros2_ws), you can now build ros2_rplidar_sub package using the command:

```bash
cd ..
colcon build --symlink-install
```

## Run ros2_rplidar_sub

**NOTE** - Before running package `ros2_rplidar_sub` please launch package `sllidar_ros2` for getting ranges.

[For Detailed Instructions on `sllidar_ros2` Click here](https://github.com/dhanushshettigar/ros-raspberry-pi/blob/main/RPLidar/RPLIDAR.md)

**Step 5** Package environment setup

```bash
source install/setup.bash
```

**Step 6** Run sllidar_ros2 node

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

**Step 7** Open a new terminal:

*Replace `<container_id>` with copied container id

```bash
docker exec -it <container_id> /bin/bash
```

**Step 8** Source the setup files
```bash
source /opt/ros/iron/setup.bash
```

**Step 9** Plot RPLiDAR ranges.

```bash
ros2 run ros2_rplidar_sub scan_listener
```

After all measurements have been plotted, the display is updated.
