#include <cstdio>

#include "sr_core/referee_serial.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sim_referee::core::SimReferee>("/dev/ttyUSB0", 1000ms));
    rclcpp::shutdown();
    return 0;
}
