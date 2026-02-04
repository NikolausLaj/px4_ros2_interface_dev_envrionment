#include "rclcpp/rclcpp.hpp"

#include "pieps_interfacer/msg/pieps_measurements.hpp"


class FluxlineFollowController : public rclcpp::Node
{
    public:
        FluxlineFollowController();
        ~FluxlineFollowController();
    private:
        rclcpp::Subscription<pieps_interfacer::msg::PiepsMeasurements>::SharedPtr pieps_sub_;
        void piepsCallback(const pieps_interfacer::msg::PiepsMeasurements &msg);
};

