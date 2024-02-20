#include "header.h"
#include "vehicle_interfaces/params.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("controlserver_params_node");
    auto controller = std::make_shared<Controller>(params);
    rclcpp::executors::SingleThreadedExecutor* executor = new rclcpp::executors::SingleThreadedExecutor();
    executor->add_node(controller);
    std::thread execTh(vehicle_interfaces::SpinExecutor, executor, "controller", 1000.0);

    while (1)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    executor->cancel();
    execTh.join();
    controller->close();
    delete executor;
    rclcpp::shutdown();
    return 0;
}