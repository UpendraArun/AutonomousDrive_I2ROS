# Team15 Cars Autonomous Driving Project

This project enables a vehicle to drive autonomously in simulation, obeying traffic signals and minimizing travel time. Built with ROS Noetic.

## Getting Started

1. **Install Dependencies** (see below).
2. Copy the `src` folder to your repository and build it.
3. [Download the Unity Environment](https://syncandshare.lrz.de/getlink/fiEg9ocZ6Pc5iuEa4QqN1b/).
4. Unzip and copy the Unity files to `.../devel/lib/simulation/`.
5. Source the `setup.bash` file.
6. Launch the simulation:
  ```bash
  roslaunch path_planner movebase_teb.launch
  ```
  - Multiple monitoring windows will open.
  - The vehicle will generate velocity commands that obey traffic signals.
  - Manual control is available via `w`, `a`, `s`, `d` in Unity.

7. **Check velocity commands:**
  ```bash
  rostopic echo /cmd_mutipller_node/ackermann_cmd_mux
  ```

## Dependencies Installation

```bash
sudo apt-get install ros-noetic-ackermann-msgs \
   ros-noetic-vision-opencv \
   ros-noetic-teb-local-planner \
   ros-noetic-octomap-ros \
   ros-noetic-cv-bridge \
   ros-noetic-octomap-server \
   ros-noetic-sensor-msgs \
   ros-noetic-eigen-conversions \
   ros-noetic-tf-conversions \
   ros-noetic-move-base \
   ros-noetic-move-base-msgs \
   ros-noetic-geometry-msgs \
   ros-noetic-tf2-ros \
   ros-noetic-actionlib-msgs \
   ros-noetic-message-generation \
   ros-noetic-nav-msgs
```

<div>
	<img src="https://github.com/UpendraArun/AutonomousDrive_I2ROS/blob/main/AutonomousDriving/assets/TF.png" width="33%" />
	<p><em>Point cloud and voxel grid generation</em></p>
</div>

<div>
	<img src="https://github.com/UpendraArun/AutonomousDrive_I2ROS/blob/main/AutonomousDriving/assets/OccMap.png" width="33%" />
	<p><em>3D Occupancy grid and projected map</em></p>
</div>

<div>
	<img src="https://github.com/UpendraArun/AutonomousDrive_I2ROS/blob/main/AutonomousDriving/assets/AutonomousDrive.png" width="33%" />
	<p><em>Autonomous Drive</em></p>
</div>
