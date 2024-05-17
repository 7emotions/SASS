#pragma once
/**
0x0001
0x0003
0x020c
0x020e
*/
#include <cstdint>
#include <sys/cdefs.h>
namespace sim_referee::msgs {

typedef struct {
    uint8_t game_type     : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

typedef struct {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} game_robot_HP_t;

typedef struct {
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
} radar_mark_data_t;

typedef struct {
    uint8_t radar_info;
} radar_info_t;

typedef struct frame_header {
    uint8_t sof = 0xa5;
    uint16_t data_length;
    uint8_t seq = 0x00;
    uint8_t crc8;
} frame_header;

template <class T>
struct package {
    frame_header header;
    uint16_t cmd_id;
    T body;
    uint16_t crc;
};

} // namespace sim_referee::msgs