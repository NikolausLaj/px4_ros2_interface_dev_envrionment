#include "TerrainFollowControllerNode.hpp"


using std::placeholders::_1;


TerrainFollowController::TerrainFollowController() : Node("terrain_follow_controller_node")
{
    getParameters();

    // Subscribers
    _vehicle_local_position_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", rclcpp::QoS(1).best_effort(),
        std::bind(&TerrainFollowController::vehicleLocalPositionCallback, this, std::placeholders::_1));

    _vehicle_attitude_sub = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", rclcpp::QoS(1).best_effort(),
        std::bind(&TerrainFollowController::vehicleAttitudeCallback, this, std::placeholders::_1));
    

    // Publishers
    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        "/terrain_controller/cmd_vel", 10);

    RCLCPP_INFO(get_logger(), "Terrain Follow Controller Ininitalized!");
}


void TerrainFollowController::getParameters()
{
    this->declare_parameter("target_distance", 4.0);
    this->declare_parameter("kp", 2.250);
    this->declare_parameter("ki", 0.2);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("max_vel", 3.0);
    this->declare_parameter("min_vel", -3.0);
    this->declare_parameter("integrator_limit", 2.0);
    this->declare_parameter("compensate_angle", false);

    _target_alt = this->get_parameter("target_distance").as_double();
    _kp = this->get_parameter("kp").as_double();
    _ki = this->get_parameter("ki").as_double();
    _kd = this->get_parameter("kd").as_double();
    _max_vel = this->get_parameter("max_vel").as_double();
    _min_vel = this->get_parameter("min_vel").as_double();
    _integrator_limit = this->get_parameter("integrator_limit").as_double();
    _compensate_angle = this->get_parameter("compensate_angle").as_bool();

    RCLCPP_INFO(this->get_logger(), "\nParameters Loaded:\n - Target Alt.: %f [m]\n - PID Values: Kp = %f, Ki = %f, Kp = %f\n - Max.-Vel.: %f [m/s]\n - Min.-Vel.: %f [m/s]",
                _target_alt, _kp, _ki, _kd, _max_vel, _min_vel);

}


void TerrainFollowController::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    // TODO: Set Kp accroring to V_xy max, check for mapping of distanct->velocity
    // TODO: Add Distance to bottom validity check
    geometry_msgs::msg::Twist cmd_vel_msg;
    
    auto current_call_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>( current_call_time - _last_call_time ).count();
    double control_signal = 0.0;
    
    double distance = compensateRollAndPitch(msg->dist_bottom);
    _error = _target_alt - distance;
    
    // Create PID Function
    // Propotional
    control_signal += _kp * _error; 

    // Integral
    if (std::abs(_error) < _integrator_limit)
    {
        _integral += _error * dt;
        control_signal += _ki * _integral;
    }
    
    // Derivative
    double derivative = ( _error - _last_error ) / dt;
    control_signal+= _kd * derivative;

    // Limiter
    if ( control_signal > _max_vel )
    {
        control_signal = _max_vel;
    }
    else if ( control_signal < _min_vel )
    {
        control_signal = _min_vel;
    }

    cmd_vel_msg.linear.z = -control_signal; // PX4 used NED Frame
    _cmd_vel_pub->publish(cmd_vel_msg);
    _last_call_time = current_call_time;
    _last_error = _error;
}


void TerrainFollowController::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
    tf2::Matrix3x3 m(q);
    m.getRPY(_roll, _pitch, _yaw);
}


double TerrainFollowController::compensateRollAndPitch(const float dist_measured)
{
    if (_compensate_angle)
    {
        double dist_corrected = dist_measured * std::cos(_roll) * std::cos(_pitch);
        RCLCPP_INFO(this->get_logger(), "Difference %f", dist_measured-dist_corrected);
        return dist_corrected;
    }
    return dist_measured;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TerrainFollowController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}