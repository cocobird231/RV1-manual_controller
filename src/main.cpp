#include "header.h"
#include "vehicle_interfaces/params.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("controlserver_params_node");
    auto controller = std::make_shared<Controller>(params);
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}