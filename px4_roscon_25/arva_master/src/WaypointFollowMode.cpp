#include "WaypointFollowMode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

static const std::string kModeNameWaypointFollow = "WaypointFollow";
static const std::string kModeNameCustomAltitude = "CustomAltitude";

WaypointFollow::WaypointFollow(rclcpp::Node &node)
    : px4_ros2::ModeBase(node, kModeNameWaypointFollow),
      _node(node)
{
    loadParameters();

    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    node.create_subscription<geometry_msgs::msg::Twist>(
        "/terrain_controller/cmd_vel", rclcpp::QoS(1).best_effort(),
        [this](const std::shared_ptr<const geometry_msgs::msg::Twist>& msg)
        {
            _cmd_z_vel = msg->linear.z;
        }
    );
    
    RCLCPP_INFO(node.get_logger(), "WaypointFollow mode initialized.");
}

// ----------------------------------------------------------------------------

void WaypointFollow::loadParameters()
{
    // Load parameters specific to the WaypointFollow mode
}

// ----------------------------------------------------------------------------

void WaypointFollow::onActivate()
{
    // TODO: Find a way how to get uploaded mission to companion computer
    // Initialize waypoints of Baylands
    _trajectory_waypoints.push_back(Eigen::Vector3f(78.8f,76.2f, -1.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(201.3f,	343.2f, -1.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(63.4f, 49.7f, -1.5f));

    _current_waypoint_index = 0; // Start at the first waypoint
    RCLCPP_WARN(_node.get_logger(), "WaypointFollow mode activated");
    // Set initial trajectory setpoint
}

// ----------------------------------------------------------------------------

void WaypointFollow::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "WaypointFollow mode deactivated");
    // Reset trajectory setpoint
}

// ----------------------------------------------------------------------------

void WaypointFollow::updateSetpoint([[maybe_unused]] float dt_s)
{
    if (_current_waypoint_index < _trajectory_waypoints.size()) {
        // Set the trajectory setpoint to the current waypoint
        auto current_waypoint = _trajectory_waypoints[_current_waypoint_index];

        // set current Waypoint and terrain following velocity
        _setpoint
            .withPositionX(current_waypoint.x())
            .withPositionY(current_waypoint.y())
            .withVelocityZ(_cmd_z_vel);
        _trajectory_setpoint->update(_setpoint);

        RCLCPP_INFO(_node.get_logger(), "Lat.: %f, Lon.: %f, Z-Vel.: %f", current_waypoint.x(), current_waypoint.y(), _cmd_z_vel);

        // Check if we reached the current waypoint
        if ((_local_position->positionNed() - current_waypoint).norm() < 0.5f) {
            _current_waypoint_index++; // Move to the next waypoint
        }
    } else {
        // All waypoints completed, reset or stop
        RCLCPP_INFO(_node.get_logger(), "All waypoints completed.");
        completed(px4_ros2::Result::Success);
        return; // Exit the update loop
    }
}