#include "FluxlineFollowMode.hpp"
#include <px4_ros2/components/node_with_mode.hpp>

static const std::string kModeNameFluxlineFollow = "FluxlineFollow";

FluxlineFollow::FluxlineFollow(rclcpp::Node &node) : px4_ros2::ModeBase(node, kModeNameFluxlineFollow), _node(node)
{
    loadParameters();

    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    _terrain_cmd_vel = node.create_subscription<geometry_msgs::msg::Twist>(
        "/terrain_controller/cmd_vel", rclcpp::QoS(1).best_effort(),
        std::bind(&FluxlineFollow::terrainFollowCallback, this, std::placeholders::_1)
    );

    _flux_cmd_vel = node.create_subscription<geometry_msgs::msg::Twist>(
        "/fluxline_controller/cmd_vel", rclcpp::QoS(1).best_effort(),
        std::bind(&FluxlineFollow::fluxlineFollowCallback, this, std::placeholders::_1)
    );
    
}

void FluxlineFollow::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "Fluxline Follow mode activated");

}

void FluxlineFollow::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "Fluxline Follow mode deactivated");

}

void FluxlineFollow::loadParameters()
{
    
}


void FluxlineFollow::updateSetpoint([[maybe_unused]] float dt_s)
{
    _setpoint = px4_ros2::TrajectorySetpoint();  // Reset setpoint
    _setpoint.withVelocityX(_cmd_x_vel)
        .withVelocityY(0.0f)
        .withVelocityZ(_cmd_z_vel)
        .withYawRate(_cmd_yaw_vel);
    
    _trajectory_setpoint->update(_setpoint);
}


void FluxlineFollow::terrainFollowCallback(const std::shared_ptr<const geometry_msgs::msg::Twist> &msg)
{
    _cmd_z_vel = msg->linear.z;
}

void FluxlineFollow::fluxlineFollowCallback(const std::shared_ptr <const geometry_msgs::msg::Twist> &msg)
{
    _cmd_x_vel = msg->linear.x;
    _cmd_yaw_vel = msg->angular.z;
}
