#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "pieps_interfacer/msg/pieps_measurements.hpp"


class FluxlineFollowController : public rclcpp::Node
{
    public:
        FluxlineFollowController();
        ~FluxlineFollowController();
    private:
        // Membervariables
        // float error_angle_, integral_angle_;
        // double dt_angle_;
        // std::chrono::steady_clock::time_point _last_call_time;
        
        // Methods
        float linearVelocityController(const float &dist_measure, const bool &dist_valid);
        float angularVelocityController(const float &angle_measure, const bool &angle_valid);
        // void PID(const float measurement, const float target, float &error, float &output, float &integral, const float integral_limit, const double dt, const float kp, const float ki);

        // Subscribers
        rclcpp::Subscription<pieps_interfacer::msg::PiepsMeasurements>::SharedPtr pieps_sub_;
        void piepsCallback(const pieps_interfacer::msg::PiepsMeasurements &msg);

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;

};

