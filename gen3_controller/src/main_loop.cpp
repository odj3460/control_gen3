
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <chrono> // std::chrono::milliseconds 추가


#include "gen3_controller/gen3_controller.h"
#include "gen3_controller/gen3_interface.h"

#define TorqueControl 1
#define PositionControl 0

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gen3_controller");
    DataContainer dc;
    dc.node = node;

    auto gen3_interface = std::make_shared<Gen3Interface>(node, dc);
    auto gen3_controller = std::make_shared<Gen3Controller>(node, dc);

    std::thread thread[5];
    thread[0] = std::thread(&Gen3Interface::stateUpdate, gen3_interface);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    thread[1] = std::thread(&Gen3Controller::compute, gen3_controller);
    thread[2] = std::thread(&Gen3Interface::gen3Command, gen3_interface);
    thread[3] = std::thread(&Gen3Interface::pubCommand, gen3_interface);

    for (int i = 0; i < 4; ++i)
    {
        thread[i].join();
    }

    rclcpp::shutdown();

    return 0;
}