#include "TerrainFollowControllerNode.hpp"


using std::placeholders::_1;


TerrainFollowController::TerrainFollowController() : Node("terrain_follow_controller_node")
{
    getParameters();

    // Subscribers
    _vehicle_local_position_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", rclcpp::QoS(1).best_effort(),
        std::bind(&TerrainFollowController::vehicleLocalPositionCallback, this, std::placeholders::_1));

    // Publishers
    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        "/terrain_controller/cmd_vel", 10);

    RCLCPP_INFO(get_logger(), "Terrain Follow Controller Ininitalized!");
}


void TerrainFollowController::getParameters()
{
    this->declare_parameter("target_distance", 3.0);
    this->declare_parameter("kp", 0.7);
    this->declare_parameter("ki", 0.1);
    this->declare_parameter("kd", 0.05);
    this->declare_parameter("max_vel", 2.0);
    this->declare_parameter("min_vel", -2.0);

    _target_alt = this->get_parameter("target_distance").as_double();
    _kp = this->get_parameter("kp").as_double();
    _ki = this->get_parameter("ki").as_double();
    _kd = this->get_parameter("kd").as_double();
    _max_vel = this->get_parameter("max_vel").as_double();
    _min_vel = this->get_parameter("min_vel").as_double();

    RCLCPP_INFO(this->get_logger(), "\nParameters Loaded:\n - Target Alt.: %f [m]\n - PID Values: Kp = %f, Ki = %f, Kp = %f\n - Max.-Vel.: %f [m/s]\n - Min.-Vel.: %f [m/s]",
                _target_alt, _kp, _ki, _kd, _max_vel, _min_vel);

}


void TerrainFollowController::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    geometry_msgs::msg::Twist cmd_vel_msg;
    
    auto current_call_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>( current_call_time - _last_call_time ).count();
    
    _error = _target_alt - msg->dist_bottom ;
    _integral += _error * dt;
    double derivative = ( _error - _last_error ) / dt;
    
    double control_signal = - _kp * _error; // + _ki * _integral + _kd * derivative;

    if ( control_signal > _max_vel )
    {
        control_signal = _max_vel;
    }
    else if ( control_signal < _min_vel )
    {
        control_signal = _min_vel;
    }

    cmd_vel_msg.linear.z = control_signal;
    _cmd_vel_pub->publish(cmd_vel_msg);
    _last_call_time = current_call_time;
    _last_error = _error;

    RCLCPP_INFO(this->get_logger(), "Measure Dist.: %f, Error: %f, Controll Sig.: %f", msg->dist_bottom, _error, control_signal);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TerrainFollowController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}