#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

#include "pieps_interfacer/msg/pieps_measurements.hpp"


class FluxlineFollowController : public rclcpp::Node
{
    public:
        FluxlineFollowController();
        ~FluxlineFollowController();
    private:
        float last_error_angle_ = 0.0;
        double kp_angle_, kd_angle_, target_angle_, angular_vel_limit_;
        std::chrono::steady_clock::time_point last_call_angle_;

        // Methods
        void loadParameters();
        float linearVelocityController(const float &dist_measure, const bool &dist_valid);
        float angularVelocityController(const float &angle_measure, const bool &angle_valid);
        float timeDiff(std::chrono::steady_clock::time_point &last_call);
        void limiter(float &value, const float limit);

        // Subscribers
        rclcpp::Subscription<pieps_interfacer::msg::PiepsMeasurements>::SharedPtr pieps_sub_;
        void piepsCallback(const pieps_interfacer::msg::PiepsMeasurements &msg);

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;

};

