#ifndef IMU_H
#define IMU_H

#include <iostream>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <unistd.h>
#include <thread> 
#include <chrono>
#include <sys/ioctl.h> 
#include <fcntl.h>
#include "register_map.h"
#include <boost/log/trivial.hpp>
#include <boost/signals2/signal.hpp>

class Imu_Interface {
private:
    int fd;
    void publish_mag_xyz();   
    void publish_acc_xyz(); 
    void publish_gyr_xyz(); 
    void publish_lin_acc_xyz(); 
    void publish_pitch(); 
    void publish_roll(); 
    void publish_head();  
    void publish_eul_rph(); 
    void publish_qua_xyzw(); 
    int16_t read_16_bit_reg(const uint8_t& reg_l, const uint8_t& reg_h, const int& fd); 
    bool check_calibration(); // New function to verify calibration

    const float QUAT_SCALE_VAL = 16384.0; 
    const float EUL_SCALE_VAL = 0.0625; 
    const float ACC_SCALE_VAL = 0.01; 
    const float GYR_SCALE_VAL = 0.0625; 
 

public:
    Imu_Interface(const char* file_addr, uint8_t slave_addr_param); 
    ~Imu_Interface(); 

    void initialize_read_loop(int delay;  
    boost::signals2::signal<void(std::array<int16_t,3> mag_xyz_raw)> mag_signal_raw; 
    boost::signals2::signal<void(std::array<int16_t,3> acc_xyz_raw)> acc_signal_raw; 
    boost::signals2::signal<void(std::array<int16_t,3> lin_acc_xyz_raw)> lin_acc_signal_raw; 
    boost::signals2::signal<void(std::array<int16_t,3> gyr_xyz_raw)> gyr_signal_raw; 
    boost::signals2::signal<void(std::array<int16_t,4> qua_xyzw_raw)> qua_signal_raw; 
    boost::signals2::signal<void(std::array<int16_t,3> eul_rph_raw)> eul_signal_raw; 

    boost::signals2::signal<void(std::array<double,3> mag_xyz_scaled)> mag_signal_scaled; 
    boost::signals2::signal<void(std::array<double,3> acc_xyz_scaled)> acc_signal_scaled; 
    boost::signals2::signal<void(std::array<double,3> lin_acc_xyz_scaled)> lin_acc_signal_scaled; 
    boost::signals2::signal<void(std::array<double,3> gyr_xyz_scaled)> gyr_signal_scaled; 
     boost::signals2::signal<void(std::array<double,4> qua_xyzw_scaled)> qua_signal_scaled; 
    boost::signals2::signal<void(std::array<double,3> eul_rph_scaled)> eul_signal_scaled; 

    boost::signals2::signal<void(double roll)> roll_signal; 
    boost::signals2::signal<void(double pitch)> pitch_signal; 
    boost::signals2::signal<void(double head)> head_signal; 
    
};

#endif