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

    _terrain_cmd_vel = node.create_subscription<geometry_msgs::msg::Twist>(
        "/terrain_controller/cmd_vel", rclcpp::QoS(1).best_effort(),
        std::bind(&WaypointFollow::terrainFollowCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(node.get_logger(), "WaypointFollow mode initialized.");
}


void WaypointFollow::loadParameters()
{
    // Load parameters specific to the WaypointFollow mode
}


void WaypointFollow::onActivate()
{
    // TODO: Find a way how to get uploaded mission to companion computer
    // Initialize waypoints of Baylands

    // _trajectory_waypoints.push_back(Eigen::Vector3f(10.0f, 0.0f, -1.5f));
    // _trajectory_waypoints.push_back(Eigen::Vector3f(10.0f,	10.0f, -1.5f));
    // _trajectory_waypoints.push_back(Eigen::Vector3f(0.0f, 10.0f, -1.5f));
    // _trajectory_waypoints.push_back(Eigen::Vector3f(0.0f, 0.0f, -1.5f));

    // Baylands Coordinates Long
    // _trajectory_waypoints.push_back(Eigen::Vector3f(78.8f,76.2f, -1.5f));
    // _trajectory_waypoints.push_back(Eigen::Vector3f(201.3f,	343.2f, -1.5f));
    // _trajectory_waypoints.push_back(Eigen::Vector3f(63.4f, 49.7f, -1.5f));

    // Baylands Coordinates Short
    _trajectory_waypoints.push_back(Eigen::Vector3f(65.57, 55.94, -1.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(83.36, 89.91, -1.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(116.56, 156.73, -1.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(83.36, 89.91, -1.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(116.56, 156.73, -1.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(65.57, 55.94, -1.5f));
    _trajectory_waypoints.push_back(Eigen::Vector3f(0.0, 0.0, -1.5f));

    _current_waypoint_index = 0; // Start at the first waypoint
    RCLCPP_WARN(_node.get_logger(), "WaypointFollow mode activated");
    // Set initial trajectory setpoint
}


void WaypointFollow::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "WaypointFollow mode deactivated");
    // Reset trajectory setpoint
}


void WaypointFollow::updateSetpoint([[maybe_unused]] float dt_s)
{
    // TODO Find a way to limit flight speed. Or not keep it by the UI and global PX4 Settings
    if (_current_waypoint_index < _trajectory_waypoints.size())
    {
        auto current_waypoint = _trajectory_waypoints[_current_waypoint_index];

        _setpoint
            .withHorizontalPosition(Eigen::Vector2f {current_waypoint.x(), current_waypoint.y()})
            .withVelocityZ(_cmd_z_vel);
        _trajectory_setpoint->update(_setpoint);

        if (checkIfReached(current_waypoint))
        {
            _current_waypoint_index++; // Move to the next waypoint
        }
    }
    else
    {
        // All waypoints completed, reset or stop
        RCLCPP_INFO(_node.get_logger(), "All waypoints completed.");
        completed(px4_ros2::Result::Success);
        return; // Exit the update loop
    }
}


bool WaypointFollow::checkIfReached(Eigen::Vector3f &current_waypoint) const
{
    Eigen::Vector3f position = _local_position->positionNed();
    Eigen::Vector2f current_position{ position[0], position[1] };
    Eigen::Vector2f goal_position{ current_waypoint.x(), current_waypoint.y() };

    if ((goal_position - current_position).norm() < 0.5)
    {
        return true;
    }
    return false;
}


void WaypointFollow::terrainFollowCallback(const std::shared_ptr<const geometry_msgs::msg::Twist> &msg)
{
    _cmd_z_vel = msg->linear.z;
}