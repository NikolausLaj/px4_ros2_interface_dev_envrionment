// ============================================================================
// ORIGINAL VERSION - CustomMode.hpp
// ============================================================================
#pragma once

// PX4 Interface Library
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/angular_velocity.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

// ROS 2 Core
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

// C++ Std
#include <cmath>  // for M_PI
#include <Eigen/Eigen>
#include <chrono>

// ---------------------------------------------------------------------------------------------

class WaypointFollow : public px4_ros2::ModeBase {
    public:
        explicit WaypointFollow(rclcpp::Node &node);

        // See ModeBase
        void onActivate() override;
        void onDeactivate() override;
        void updateSetpoint([[maybe_unused]] float dt_s) override;

    private:
        // Member Var
        double _cmd_z_vel;
        rclcpp::Node &_node;
        px4_ros2::TrajectorySetpoint _setpoint;
        std::vector<Eigen::Vector3f> _trajectory_waypoints; // Vector to hold waypoints
        size_t _current_waypoint_index; // Index of the current waypoint     
        std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
        std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;

        // Subscriber
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _terrain_cmd_vel;
        
        void loadParameters();
};