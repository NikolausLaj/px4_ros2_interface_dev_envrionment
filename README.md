<h1 align="center">Hands-On Aerial Robotics Using PX4 and ROS 2</h1>

<p align="center">
    <strong>Fly with ROS 2. Powered by PX4</strong>
</p>
<p align="center">
    <a href="https://creativecommons.org/licenses/by-sa/4.0/">
        <img src="https://img.shields.io/badge/License-CC_BY--SA_4.0-lightgrey.svg" alt="License: CC BY-SA 4.0">
    </a>
</p>

## Launch
1. Gazebo and Baylands World
    ```console
    python3 /home/ubuntu/PX4-gazebo-models/simulation-gazebo --model_store /home/ubuntu/PX4-gazebo-models/ --world baylands
    ```

2. Model Lidar Down: 4016
    ```console
    PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4016 PX4_PARAM_UXRCE_DDS_SYNCT=0 /home/ubuntu/px4_sitl/bin/px4 -w /home/ubuntu/px4_sitl/romfs
    ```

3. QGC:
    ```concole
    /home/ubuntu/QGroundControl/qgroundcontrol
    ```

4. Common Launch:
    ```concole
    ros2 launch px4_roscon_25 common.launch.py
    ```

5. Custom Code:
    ```concole
    ros2 launch custom_mode_demo custom_mode_demo.launch.py
    ```
  


## About

Picture yourself in Gazebo, sending velocity, position, and acceleration commands straight from your ROS 2 nodes while PX4 quietly manages real-time flight control and built-in safety.
Over the course of a full-day, hands-on workshop, you'll see why PX4 powers so many high-reliability robotics projects, learn to hook your existing ROS 2 setup into our plug-and-play bridge, and stream live telemetry back to your code.
You'll explore perception integrations, think ArUco markers and LiDAR, build high-level control routines in ROS 2, and leave with working simulation environments, example projects, and a clear path to swap out virtual drones for real hardware.
No PX4 background is needed, just your ROS expertise, a laptop, and a spirit of discovery. Everything in this workshop is fully open source and open for you to inspect, modify, and share.

This repository contains all the materials for the ROSCon 2025, Hands-On Aerial Robotics Using PX4 and ROS 2 Workshop.
For questions regarding the workshop, please join the Dronecode Foundation Discord and post your question in the workshop-roscon-2025 channel:

[Join the Dronecode Foundation Discord](https://chat.dronecode.org/)

## Outline

This workshop introduces you to PX4’s ROS 2 integration layer and shows how to create your own flight modes, perception pipelines, and control executors.
You’ll also get a brief introduction to PX4 and a ready-to-use developer environment designed for seamless integration between PX4 and ROS 2.
By the end of the workshop, you’ll have a complete ROS 2 package capable of controlling a simulated drone, performing navigation tasks, and executing precision landings.

**Note:** Each example includes several exercises designed to reinforce the concepts, deepen understanding, and encourage exploration of the environment.
Solutions are provided either **commented out in the code** (you can uncomment and rebuild the package to test) or as **separate, individual packages.**

For more detailed instructions and guidance, please refer to the dedicated **README** for each example.

### Presentation

[Link to Presentation on Google Slides](https://docs.google.com/presentation/d/1S0erGP3pqjSlPU8--NCr8zwdNYxXy2enIBOqPs76fCQ/edit?usp=sharing)

### Introduction & Drone Architecture

[PX4 Documentation](https://docs.px4.io/main/en/)

### Environment Setup

For detailed environment and Docker setup instructions, see the [docs/README.md](docs/setup.md) guide.
Please complete this step before you proceed.

### Control Pipelines

There are two main ways to interact with PX4 and ROS 2:

**Offboard Mode** – the classic method for sending velocity or position setpoints directly to PX4.

**PX4 ROS 2 Interface / Custom Modes** – the newer method using the px4_ros2 library, allowing you to create custom flight modes and executors.

In this section, we demonstrate a simple flight sequence: Takeoff → Waypoints → Yaw → Landing.
The goal is to compare these two approaches, highlighting their differences and advantages.

For detailed instructions and exercises, refer to the following guides in this repository:

- [Offboard Demo](px4_roscon_25/offboard_demo/README.md)
- [Custom Mode Demo](px4_roscon_25/custom_mode_demo/README.md)

### Perception & Applications

In this section, we explore **three practical examples** of perception and control in ROS 2 with PX4:

1. **ArUco Marker Detection** – Detect markers using ROS 2 and PX4. No custom flight mode is required.
2. **Teleoperation** – Ever seen a TurtleBot flying? This demo shows how to manually control a drone using a keyboard and to use a LiDAR scan for environmental awareness.
3. **Precision Landing** – Combine ArUco detection with a **Custom Mode** to perform precision landing.
    - **Precision Landing with Executor** – This is a follow up exercise to incorporate Precision Land in the former Custom Modes Demo, where an Executor schedules  Waypoints and Precison Land to find and land on the ArUco Marker in the maze. 

For more detailed instructions and exercises, refer to the following demos:

- [ArUco Marker Detection](px4_roscon_25/aruco_tracker/README.md)
- [Teleoperation](px4_roscon_25/teleop/README.md)
- [Precision Landing](px4_roscon_25/precision_land/README.md)
- [Precision Landing with Executor](px4_roscon_25/precision_land_executor/README.md)

### Q&A, Resources & Hardware Show-and-Tell

At the end of the workshop, we provide dedicated time for questions, troubleshooting, and hands-on guidance with hardware.

### Open Lab & Individual Consultations

Dedicated time for leftover exercises and individual consultation.



// _trajectory_waypoints.push_back(Eigen::Vector2f(0.00f, 0.00f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-7.86f, 5.65f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-19.24f, 10.65f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-62.49f, 117.32f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-77.87f, 168.24f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-76.55f, 203.24f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-74.39f, 235.00f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-65.90f, 265.62f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-62.23f, 297.48f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-60.84f, 335.86f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-62.73f, 354.77f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-69.60f, 397.19f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-70.09f, 441.85f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-67.23f, 474.07f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-61.02f, 493.87f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-38.23f, 474.97f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(-21.55f, 460.43f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(28.30f, 446.74f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(60.23f, 429.77f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(85.62f, 398.18f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(101.85f, 367.64f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(131.19f, 346.02f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(169.86f, 334.74f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(197.99f, 332.13f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(194.95f, 308.29f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(181.91f, 283.74f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(171.71f, 262.37f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(150.02f, 218.11f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(125.95f, 171.54f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(60.06f, 37.91f));
// _trajectory_waypoints.push_back(Eigen::Vector2f(30.14f, -20.31f));
