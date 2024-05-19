# SASS
Serial Arbitration System Simulator

## How to Use

1. Check out your serial port.

``` sh
ls /dev/ttyACM* /dev/ttyUSB* 
```

2. Set up serial port name and frequncy in `sim_referee_node.cpp`.

``` cpp
rclcpp::spin(std::make_shared<sim_referee::core::SimReferee>("/dev/ttyUSB0", 1000ms / 3));
```

3. Rewrite the timer callback in `referee_serial.hpp`. Choose a message package struct in `msgs.hpp`.

``` cpp
timer_ = this->create_wall_timer(period, [this] {
    // typename T seen msgs.hpp
    msgs::package<msgs::game_status_t> package;
    // Set values Here
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

```
