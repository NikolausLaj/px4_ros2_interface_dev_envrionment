#include "FluxlineFollowControllerNode.hpp"

using std::placeholders::_1;

FluxlineFollowController::FluxlineFollowController() : Node("fluxline_follow_controller_node")
{
    // Init Subs
    pieps_sub_ = this->create_subscription<pieps_interfacer::msg::PiepsMeasurements>(
        "/pieps/measurement", 10, std::bind(&FluxlineFollowController::piepsCallback, this, _1) 
    );

    // Init Pubs
    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fluxline_controller/cmd_vel", 10);
}


FluxlineFollowController::~FluxlineFollowController()
{
    
}


void FluxlineFollowController::piepsCallback(const pieps_interfacer::msg::PiepsMeasurements &msg)
{
    geometry_msgs::msg::Twist cmd_vel_msg;
    float linar_vel = linearVelocityController(msg.distance, msg.distance_valid);
    float angular_vel = angularVelocityController(msg.angle, msg.angle_valid);
    
    cmd_vel_msg.linear.x = linar_vel;
    cmd_vel_msg.angular.z = angular_vel;
    _cmd_vel_pub->publish(cmd_vel_msg);
    
}


float FluxlineFollowController::linearVelocityController(const float &dist_measure, const bool &dist_valid)
{
    return 2.5;
}


float FluxlineFollowController::angularVelocityController(const float &angle_measure, const bool &angle_valid)
{
    return 1.5;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FluxlineFollowController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}