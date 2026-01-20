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


    _vehicle_local_position_sub = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", rclcpp::QoS(1).best_effort(), std::bind(&WaypointFollow::vehicleLocalPositionCallback, this, std::placeholders::_1)
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
    // Initialize waypoints

    _trajectory_waypoints.push_back(Eigen::Vector3f(0.0f, 2.0f,-4.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(11.0f, 2.0f, -4.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(11.0f, 7.0f, -4.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(2.0f, 7.0f, -4.5f));

    // _trajectory_waypoints.push_back(Eigen::Vector3f(0.0f, 0.0f,-1.5f));

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
        _trajectory_setpoint->updatePosition(current_waypoint);
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

void WaypointFollow::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    RCLCPP_INFO(_node.get_logger(), "%f", msg->dist_bottom) ;
}