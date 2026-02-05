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
    return 0.0;
}


float FluxlineFollowController::angularVelocityController(const float &angle_measure, const bool &angle_valid)
{
    // float integral_limit = 3.1415; // TODO Add to yaml file
    // float ki = 0.1; // TODO Add to yaml file
    float controll_signal = 0.0;
    float kp = 0.7; // TODO Add to yaml file
    float target = 0; // TODO Add to yaml file

    if (angle_valid)
    {
        // auto current_call_time = std::chrono::steady_clock::now();
        // dt_angle_ = std::chrono::duration<double>( current_call_time - _last_call_time ).count();
        // PID(angle_measure, target, error_angle_, controll_signal, integral_angle_, integral_limit, dt_angle_, kp, ki);
        // _last_call_time = current_call_time;
        float error = target - angle_measure;
        controll_signal = kp * error; 
        return controll_signal;
    }
    return 0.0;
}


// void FluxlineFollowController::PID(const float measurement, const float target, float &error, float &output, float &integral, const float integral_limit, const double dt, const float kp, const float ki)
// {
//     error = target - measurement;
//     output = kp * error; 

//     // Integral
//     if (std::abs(error) < integral_limit)
//     {
//         integral += error * dt;
//         output += ki * integral;
//     }
// }


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FluxlineFollowController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}