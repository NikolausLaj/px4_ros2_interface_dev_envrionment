#pragma once

// PX4 Interface Library
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

// ROS 2 Core
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <Eigen/Eigen>


class FluxlineFollow : public px4_ros2::ModeBase {
    public:
        // Constructor
        explicit FluxlineFollow(rclcpp::Node &node);

        // Methods
        void onActivate() override;
        void onDeactivate() override;
        void updateSetpoint([[maybe_unused]] float dt_s) override;

    private:
        // Member Variables
        rclcpp::Node &_node;
        px4_ros2::TrajectorySetpoint _setpoint;
        std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
        double _cmd_x_vel, _cmd_z_vel, _angular_vel;

        // Methods
        void loadParameters();

        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _terrain_cmd_vel;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _flux_cmd_vel;

        // Callbacks
        void terrainFollowCallback(const std::shared_ptr <const geometry_msgs::msg::Twist> &msg);

};
