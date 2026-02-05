#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "pieps_interfacer/msg/pieps_measurements.hpp"


class FluxlineFollowController : public rclcpp::Node
{
    public:
        FluxlineFollowController();
        ~FluxlineFollowController();
    private:
        // Methods
        float linearVelocityController(const float &dist_measure, const bool &dist_valid);
        float angularVelocityController(const float &angle_measure, const bool &angle_valid);

        // Subscribers
        rclcpp::Subscription<pieps_interfacer::msg::PiepsMeasurements>::SharedPtr pieps_sub_;
        void piepsCallback(const pieps_interfacer::msg::PiepsMeasurements &msg);

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;

};

