#pragma once

#include "sr_core/msgs.hpp"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <rclcpp/logging.hpp>
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
            {
                msgs::package<msgs::game_status_t> package;
                load_package(package);
                load_package_info(package);
                // RCLCPP_INFO(get_logger(), "%lu,%lu", sizeof(package.body), sizeof(package));
                std::vector<uint8_t> bytes(sizeof(package));
                memcpy(bytes.data(), &package, sizeof(package));

                auto size = serial_.write(bytes);
                RCLCPP_INFO(get_logger(), "Package sent : %zu", size);
            }
        });
    }

private:
    template <typename T>
    bool load_package_info(msgs::package<T>& package) {
        package.header.data_length = sizeof(package.body);

        serial_util::dji_crc::append_crc8(package.header);
        serial_util::dji_crc::append_crc16(package);
        return true;
    }

    static bool load_package(msgs::package<msgs::game_status_t>& package) {
        package.cmd_id = 0x0001;

        package.body.game_type         = 1;
        package.body.game_progress     = 4;
        package.body.stage_remain_time = 25;
        package.body.SyncTimeStamp     = 152313;
        return true;
    }

    static bool load_package(msgs::package<msgs::game_robot_HP_t>& package) {
        package.cmd_id = 0x0003;

        package.body.blue_1_robot_HP = 100;
        package.body.blue_2_robot_HP = 100;
        package.body.blue_3_robot_HP = 100;
        package.body.blue_4_robot_HP = 100;
        package.body.blue_5_robot_HP = 100;
        package.body.blue_7_robot_HP = 100;
        package.body.blue_base_HP    = 100;
        package.body.blue_outpost_HP = 100;
        package.body.red_1_robot_HP  = 100;
        package.body.red_2_robot_HP  = 100;
        package.body.red_3_robot_HP  = 100;
        package.body.red_4_robot_HP  = 100;
        package.body.red_5_robot_HP  = 100;
        package.body.red_7_robot_HP  = 350;
        package.body.red_base_HP     = 100;
        package.body.red_outpost_HP  = 100;
        return true;
    }

    static bool load_package(msgs::package<msgs::radar_info_t>& package) {
        package.cmd_id = 0x020e;

        package.body.radar_info = 0xff;
        return true;
    }

    static bool load_package(msgs::package<msgs::radar_mark_data_t>& package) {
        package.cmd_id = 0x020c;

        package.body.mark_engineer_progress   = 1;
        package.body.mark_hero_progress       = 2;
        package.body.mark_sentry_progress     = 3;
        package.body.mark_standard_3_progress = 4;
        package.body.mark_standard_4_progress = 5;
        package.body.mark_standard_5_progress = 6;
        return true;
    }

    size_t count_;

    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_;
};
} // namespace sim_referee::core