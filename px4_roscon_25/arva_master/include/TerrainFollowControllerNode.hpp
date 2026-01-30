#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>


class TerrainFollowController : public rclcpp::Node
{
    public:
        TerrainFollowController();

    private:
        // TODO: Add Validity Check of the Distance Measurement
        // Member Variables
        double _target_alt, _error, _integral;
        double _kp, _ki, _kd;
        double _max_vel, _min_vel, _integrator_limit;
        double _last_error = 0.0;
        bool _compensate_angle;
        std::array<float, 4> _q;

        std::chrono::steady_clock::time_point _last_call_time;


        // Methods
        void getParameters();
        double angleCompensatin(const float measured_dist);

        // Subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr _vehicle_attitude_sub;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;

        // Callbacks
        void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
        void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
};