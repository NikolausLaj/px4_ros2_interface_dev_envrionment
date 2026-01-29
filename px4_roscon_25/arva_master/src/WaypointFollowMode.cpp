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
    // _trajectory_waypoints.push_back(Eigen::Vector2f(65.57, 55.94));
    // _trajectory_waypoints.push_back(Eigen::Vector2f(83.36, 89.91));
    // _trajectory_waypoints.push_back(Eigen::Vector2f(116.56, 156.73));
    // _trajectory_waypoints.push_back(Eigen::Vector2f(83.36, 89.91));
    //     _trajectory_waypoints.push_back(Eigen::Vector2f(116.56, 156.73));
    // _trajectory_waypoints.push_back(Eigen::Vector2f(65.57, 55.94));
    // _trajectory_waypoints.push_back(Eigen::Vector2f(0.0, 0.0));

//     _trajectory_waypoints.push_back(Eigen::Vector2f(0.00f, 0.00f));
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


    _current_wp_idx = 0; // Start at the first waypoint
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
    if (_current_wp_idx < _trajectory_waypoints.size())
    {
        Eigen::Vector2f current_waypoint = _trajectory_waypoints[_current_wp_idx];

        _setpoint
            .withHorizontalPosition(current_waypoint)
            .withVelocityZ(_cmd_z_vel)
            .withYaw(_yaw_target);
            
        _trajectory_setpoint->update(_setpoint);

        if (checkIfReached())
        {
            computeYaw();
            _current_wp_idx++; // Move to the next waypoint
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


bool WaypointFollow::checkIfReached() const
{
    Eigen::Vector3f position = _local_position->positionNed();
    Eigen::Vector2f current_position{ position[0], position[1] };

    if ((_trajectory_waypoints[_current_wp_idx] - current_position).norm() < 0.5) // TODO Create Parameter for Radius
    {
        return true;
    }
    return false;
}


void WaypointFollow::computeYaw()
{
    Eigen::Vector2f new_wp = _trajectory_waypoints[_current_wp_idx+1];
    Eigen::Vector2f old_wp;
    
    if (_current_wp_idx > 0)
    {
        old_wp = _trajectory_waypoints[_current_wp_idx];
    }
    else
    {
        old_wp = {0.0, 0.0};
    }

    Eigen::Vector2f diff = new_wp - old_wp;
    _yaw_target = std::atan2(diff[1], diff[0]);
}


void WaypointFollow::terrainFollowCallback(const std::shared_ptr<const geometry_msgs::msg::Twist> &msg)
{
    _cmd_z_vel = msg->linear.z;
}