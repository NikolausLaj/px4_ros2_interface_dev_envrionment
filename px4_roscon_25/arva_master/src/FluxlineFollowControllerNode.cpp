#include "FluxlineFollowControllerNode.hpp"

using std::placeholders::_1;

FluxlineFollowController::FluxlineFollowController() : Node("fluxline_follow_controller_node")
{
    pieps_sub_ = this->create_subscription<pieps_interfacer::msg::PiepsMeasurements>(
        "/pieps/measurement", 10, std::bind(&FluxlineFollowController::piepsCallback, this, _1) 
    );
}


FluxlineFollowController::~FluxlineFollowController()
{
    
}


void FluxlineFollowController::piepsCallback(const pieps_interfacer::msg::PiepsMeasurements &msg)
{
    RCLCPP_INFO(get_logger(), "Pieps Data distance: %f, angle_valid: %s",
        msg.distance, msg.angle_valid ? "true" : "false");
    
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FluxlineFollowController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}