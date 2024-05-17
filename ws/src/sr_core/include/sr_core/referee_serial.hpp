#pragma once

#include "sr_core/msgs.hpp"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <serial/serial.h>
#include <serial_util/crc/dji_crc.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace sim_referee::core {
class SimReferee : public rclcpp::Node {
public:
    explicit SimReferee(
        const std::string& port = "/dev/ttyACM0", std::chrono::milliseconds period = 1000ms)
        : rclcpp::Node("sim_referee_node")
        , count_(0)
        , serial_(port, 115200) {
        timer_ = this->create_wall_timer(period, [this] {
            msgs::package<msgs::game_status_t> package;
            package.body.game_type         = 1;
            package.body.game_progress     = 3;
            package.body.stage_remain_time = 425;
            package.body.SyncTimeStamp     = 152313;
            package.header.data_length     = sizeof(package.body);
            serial_util::dji_crc::append_crc8(package.header);
            serial_util::dji_crc::append_crc16(package);

            std::vector<uint8_t> bytes(sizeof(package));
            memcpy(bytes.data(), &package, sizeof(package));

            serial_.write(bytes);
        });
    }

private:
    size_t count_;

    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_;
};
} // namespace sim_referee::core