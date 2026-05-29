# Trajectory-Sampling-Predictive-Controller

# TSPC Controller

This repository contains a ROS2 Python implementation of a trajectory-sampling / predictive-control style controller for F1TENTH-style autonomous driving.

The main controller combines:

- Raceline tracking
- Fan-of-trajectories candidate rollout
- LiDAR-reactive obstacle avoidance
- Gap-following-style steering and speed adjustment
- Ackermann drive command publishing
- RViz visualization markers for waypoints, reference trajectory, and candidate paths

## Repository Structure

```text
tspc_controller/
├── tspc_controller/
│   ├── tspc_controller/
│   │   ├── __init__.py
│   │   ├── tspc_node.py
│   │   └── utils.py
│   ├── package.xml
│   ├── setup.py
│   └── resource/
│       └── tspc_controller
├── launch/
│   └── tspc.launch.py
├── config/
│   └── tspc_params.yaml
├── csv_data/
│   └── README.md
├── requirements.txt
├── .gitignore
└── README.md
```

## Main Files

- `tspc_controller/tspc_controller/tspc_node.py`  
  Main ROS2 node for fan-of-trajectories predictive control with LiDAR-reactive gap analysis.

- `tspc_controller/tspc_controller/utils.py`  
  Utility functions for trajectory processing, including nearest-point search.

## ROS2 Topics

The node uses the following topics by default:

| Type | Topic |
|---|---|
| Odometry / Pose input | `/ego_racecar/odom` in simulation, `/pf/viz/inferred_pose` on real car |
| LiDAR input | `/scan` |
| Ackermann command output | `/drive` |
| Reference trajectory marker | `/ref_traj_marker` |
| Waypoints marker | `/waypoints_marker` |
| Candidate path markers | `/pred_path_marker` |

## Dependencies

This project expects ROS2 with the following message packages:

- `rclpy`
- `nav_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `ackermann_msgs`
- `visualization_msgs`

Python dependencies are listed in `requirements.txt`.

## Installation

Clone the repository into the `src` folder of your ROS2 workspace:

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/YOUR_USERNAME/tspc_controller.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Running the Node

```bash
ros2 launch tspc_controller tspc.launch.py
```

Or run directly:

```bash
ros2 run tspc_controller tspc_node
```

## Waypoint CSV Files

The controller expects raceline CSV files inside the `csv_data/` folder.  
The default map name in the code is:

```text
Catalunya_fast
```

So the expected file is:

```text
csv_data/Catalunya_fast.csv
```

Large raceline or dataset files are not included by default.

## Suggested GitHub Topics

```text
ros2
f1tenth
autonomous-driving
trajectory-sampling
predictive-control
mpc
gap-follow
lidar
ackermann
robotics
```

## License and Attribution

The `utils.py` file includes an MIT License header and credits the original authors listed in that file.  
Please keep that header if you modify or redistribute the file.

Add your preferred license for the full repository after confirming how you want to release your own controller code.
