#include "FluxlineFollowControllerNode.hpp"

using std::placeholders::_1;

FluxlineFollowController::FluxlineFollowController() : Node("fluxline_follow_controller")
{
    loadParameters();

    // Init Subs
    pieps_sub_ = this->create_subscription<pieps_interfacer::msg::PiepsMeasurements>(
        "/pieps/measurement", 10, std::bind(&FluxlineFollowController::piepsCallback, this, _1));

    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", rclcpp::QoS(1).best_effort(),
        std::bind(&FluxlineFollowController::vehicleLocalPositionCallback, this, std::placeholders::_1));

    // Init Pubs
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fluxline_controller/cmd_vel", 10);
}


FluxlineFollowController::~FluxlineFollowController()
{
    
}


void FluxlineFollowController::loadParameters()
{
    // Param Angular Controller
    this->declare_parameter("angular_controller.kp", 0.0);
    this->declare_parameter("angular_controller.kd", 0.0);
    this->declare_parameter("angular_controller.target_value", 0.0);
    this->declare_parameter("angular_controller.angular_velocity_limit", 0.0);
    
    kp_angle_ = this->get_parameter("angular_controller.kp").as_double();
    kd_angle_ = this->get_parameter("angular_controller.kd").as_double();
    target_angle_ = this->get_parameter("angular_controller.target_value").as_double();
    angular_vel_limit_ = this->get_parameter("angular_controller.angular_velocity_limit").as_double();

    RCLCPP_INFO(this->get_logger(), "\nAngular Controller Parameters Loaded:\n - Target Val.: %f [rad]\n - PD Values: Kp = %f, Kd = %f\n - Ang.-Vel.-Limit: %f [rad/s]\n",
                target_angle_, kp_angle_, kd_angle_, angular_vel_limit_);
    
    // Param Linear Controller
    this->declare_parameter("linear_controller.kp", 0.0);
    this->declare_parameter("linear_controller.ki", 0.0);
    this->declare_parameter("linear_controller.kd", 0.0);
    this->declare_parameter("linear_controller.target_value",0.0);
    this->declare_parameter("linear_controller.linear_velocity_limit",0.0);
    
    kp_lin_ = this->get_parameter("linear_controller.kp").as_double();
    ki_lin_ = this->get_parameter("linear_controller.ki").as_double();
    kd_lin_ = this->get_parameter("linear_controller.kd").as_double();
    target_lin_ = this->get_parameter("linear_controller.target_value").as_double();
    lin_vel_limit_ = this->get_parameter("linear_controller.linear_velocity_limit").as_double();

    RCLCPP_INFO(this->get_logger(), "\nLinear Controller Parameters Loaded:\n - Target Val.: %f [m]\n - PID Values: Kp = %f, Ki = %f, Kd = %f\n - Lin.-Vel.-Limit: %f [m/s]\n",
                target_lin_, kp_lin_, ki_lin_, kd_lin_, lin_vel_limit_);
}


void FluxlineFollowController::piepsCallback( const pieps_interfacer::msg::PiepsMeasurements &msg )
{
    geometry_msgs::msg::Twist cmd_vel_msg;
    PlanarVelocity planar_vel = linearVelocityController(msg.distance, msg.distance_valid);
    float angular_vel = angularVelocityController(msg.angle, msg.angle_valid);
    
    cmd_vel_msg.linear.x = planar_vel.x;
    cmd_vel_msg.linear.y = planar_vel.y;
    cmd_vel_msg.angular.z = angular_vel;
    cmd_vel_pub_->publish(cmd_vel_msg);   
}


void FluxlineFollowController::vehicleLocalPositionCallback( const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg )
{
    heading_ = msg->heading;
    heading_valid_ = msg->heading_good_for_control; // TODO use this for safty measures in Controllers
}


PlanarVelocity FluxlineFollowController::linearVelocityController( const float &dist_measure, const bool &dist_valid )
{
    // TODO Angle get negative --> drone flies backwards
    PlanarVelocity planar_vel{0.0, 0.0};

    if ( dist_valid && heading_valid_) // TODO atomic bool
    {
        double control_signal = 0.0;
        float error = target_lin_ - dist_measure;
        control_signal = kp_lin_ * error;
        planar_vel.x = control_signal * std::cos(heading_);
        planar_vel.y = control_signal * std::sin(heading_);
        return planar_vel;
    }
    return planar_vel;
}


float FluxlineFollowController::angularVelocityController( const float &angle_measure, const bool &angle_valid )
{
    float control_signal = 0.0;

    if (angle_valid)
    { // PD-Controller to correct drone angle
        double dt = timeDiff(last_call_angle_);
        float error = target_angle_ - angle_measure;
        float derivate = (error - last_error_angle_) / dt;
        control_signal = kp_angle_ * error + kd_angle_ * derivate;
        limiter(control_signal, angular_vel_limit_);
        return control_signal;
    }
    return 0.0;
}


float FluxlineFollowController::timeDiff( std::chrono::steady_clock::time_point &last_call )
{
    auto current_call_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>( current_call_time - last_call ).count();
    last_call = current_call_time;
    return dt;
}


void FluxlineFollowController::limiter( float &value, const float limit )
{
    if ( std::abs( value ) > limit )
    {
        float sign = std::copysign(1.0f, value); // 1.0 or -1.0
        value = sign * limit;
    }
}


int main( int argc, char * argv[] )
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FluxlineFollowController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}