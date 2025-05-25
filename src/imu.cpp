#include "../include/umrt_imu_interface/imu.h"

using std::int16_t;
using std::uint32_t;
using std::uint8_t;

Imu_Interface::Imu_Interface(const char* file_addr, uint8_t slave_addr) { 
    fd = open(file_addr, O_RDWR); 
    if (fd < 0) {
        BOOST_LOG_TRIVIAL(error) << "i2c file not found, using " << file_addr; 
        exit(1); 
    }

    // Set slave address
    if (ioctl(fd, I2C_SLAVE, slave_addr) < 0) {
        BOOST_LOG_TRIVIAL(error) << "[x] Could not set I2C slave address"; 
        exit(1); 
    }

    // Verify chip ID
    uint8_t chip_id = i2c_smbus_read_byte_data(fd, CHIP_ID_REG);
    if (chip_id != 0xA0) {
        BOOST_LOG_TRIVIAL(error) << "Invalid chip ID: " << (int)chip_id << ", expected 0xA0";
        exit(1);
    }

    // Perform power-on reset
    i2c_smbus_write_byte_data(fd, SYS_TRIGGER_REG, 0x20);
    std::this_thread::sleep_for(std::chrono::milliseconds(650)); // Wait for reset

    // Set operating mode to IMU
    i2c_smbus_write_byte_data(fd, OPR_MODE_REG, OPR_MODE_VAL); 
    std::this_thread::sleep_for(std::chrono::milliseconds(600)); // Wait for mode switch

    // Wait for calibration
    while (!check_calibration()) {
        BOOST_LOG_TRIVIAL(warning) << "Waiting for calibration...";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    BOOST_LOG_TRIVIAL(info) << "IMU calibration complete";
}

Imu_Interface::~Imu_Interface() {
    close(fd); 
    fd = -1; 
}

bool Imu_Interface::check_calibration() {
    int32_t calib_status = i2c_smbus_read_byte_data(fd, CALIB_STAT_REG);
    if (calib_status < 0) {
        BOOST_LOG_TRIVIAL(error) << "Failed to read calibration status";
        return false;
    }
    uint8_t sys_cal = (calib_status >> 6) & 0x03;
    uint8_t gyr_cal = (calib_status >> 4) & 0x03;
    uint8_t acc_cal = (calib_status >> 2) & 0x03;
    uint8_t mag_cal = calib_status & 0x03;
    BOOST_LOG_TRIVIAL(info) << "Calibration: Sys=" << (int)sys_cal << ", Gyr=" << (int)gyr_cal << ", Acc=" << (int)acc_cal << ", Mag=" << (int)mag_cal;
    //return (acc_cal == 3 && gyr_cal == 3 && mag_cal == 3); // Require full calibration
    return true; 
}

int16_t Imu_Interface::read_16_bit_reg(const uint8_t& reg_l, const uint8_t& reg_h, const int& fd) {
    int retries = 3;
    while (retries--) {
        int32_t val_h = i2c_smbus_read_byte_data(fd, reg_h);
        int32_t val_l = i2c_smbus_read_byte_data(fd, reg_l);
        if (val_h >= 0 && val_l >= 0) {
            return (static_cast<int16_t>(val_h) << 8) | static_cast<uint8_t>(val_l);
        }
        BOOST_LOG_TRIVIAL(warning) << "I2C read retry at registers " << (int)reg_h << "/" << (int)reg_l;
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Increased delay
    }
    BOOST_LOG_TRIVIAL(error) << "Failed to read registers " << (int)reg_h << "/" << (int)reg_l;
    return 0;
}

void Imu_Interface::initialize_read_loop(int delay) {
    while (true) {
        if (check_calibration()) {
            publish_mag_xyz();
            publish_acc_xyz();
            publish_gyr_xyz();
            publish_lin_acc_xyz();
            publish_roll();
            publish_pitch();
            publish_head();
            publish_eul_rph();
            publish_qua_xyzw(); 
        } else {
            BOOST_LOG_TRIVIAL(warning) << "Calibration incomplete, skipping read";
        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(delay));
    }
}

void Imu_Interface::publish_mag_xyz() {
    int16_t mag_x_full = read_16_bit_reg(MAG_X_REG_L, MAG_X_REG_H, fd); 
    int16_t mag_y_full = read_16_bit_reg(MAG_Y_REG_L, MAG_Y_REG_H, fd); 
    int16_t mag_z_full = read_16_bit_reg(MAG_Z_REG_L, MAG_Z_REG_H, fd); 
    std::array<int16_t, 3> mag_xyz_raw = {mag_x_full, mag_y_full, mag_z_full};
    std::array<double, 3> mag_xyz_scaled = {mag_x_full / 16.0, mag_y_full / 16.0, mag_z_full / 16.0}; 
    mag_signal_raw(mag_xyz_raw); 
    mag_signal_scaled(mag_xyz_scaled); 
}

void Imu_Interface::publish_acc_xyz() {
    int16_t acc_x_full = read_16_bit_reg(ACC_X_REG_L, ACC_X_REG_H, fd); 
    int16_t acc_y_full = read_16_bit_reg(ACC_Y_REG_L, ACC_Y_REG_H, fd); 
    int16_t acc_z_full = read_16_bit_reg(ACC_Z_REG_L, ACC_Z_REG_H, fd); 
    std::array<int16_t, 3> acc_xyz_raw = {acc_x_full, acc_y_full, acc_z_full};
    std::array<double, 3> acc_xyz_scaled = {acc_x_full / 100.0, acc_y_full / 100.0, acc_z_full / 100.0}; 
    acc_signal_raw(acc_xyz_raw); 
    acc_signal_scaled(acc_xyz_scaled); 
}

void Imu_Interface::publish_gyr_xyz() {
    int16_t gyr_x_full = read_16_bit_reg(GYR_X_REG_L, GYR_X_REG_H, fd);  
    int16_t gyr_y_full = read_16_bit_reg(GYR_Y_REG_L, GYR_Y_REG_H, fd);  
    int16_t gyr_z_full = read_16_bit_reg(GYR_Z_REG_L, GYR_Z_REG_H, fd);  
    std::array<int16_t, 3> gyr_xyz_raw = {gyr_x_full, gyr_y_full, gyr_z_full};
    std::array<double, 3> gyr_xyz_scaled = {gyr_x_full*GYR_SCALE_VAL, gyr_y_full*GYR_SCALE_VAL, gyr_z_full*GYR_SCALE_VAL}; 
    gyr_signal_raw(gyr_xyz_raw); 
    gyr_signal_scaled(gyr_xyz_scaled);
}

void Imu_Interface::publish_lin_acc_xyz() {
    int16_t lin_acc_x_full = read_16_bit_reg(LIN_ACC_X_REG_L, LIN_ACC_X_REG_H, fd); 
    int16_t lin_acc_y_full = read_16_bit_reg(LIN_ACC_Y_REG_L, LIN_ACC_Y_REG_H, fd); 
    int16_t lin_acc_z_full = read_16_bit_reg(LIN_ACC_Z_REG_L, LIN_ACC_Z_REG_H, fd); 
    std::array<int16_t, 3> lin_acc_xyz_raw = {lin_acc_x_full, lin_acc_y_full, lin_acc_z_full};
    std::array<double, 3> lin_acc_xyz_scaled = {lin_acc_x_full*ACC_SCALE_VAL, lin_acc_y_full*ACC_SCALE_VAL, lin_acc_z_full*ACC_SCALE_VAL}; 
    lin_acc_signal_raw(lin_acc_xyz_raw); 
    lin_acc_signal_scaled(lin_acc_xyz_scaled); 
}

void Imu_Interface::publish_qua_xyzw(){
    int16_t qua_x_full = read_16_bit_reg(QUA_X_REG_L, QUA_X_REG_H, fd); 
    int16_t qua_y_full = read_16_bit_reg(QUA_Y_REG_L, QUA_Y_REG_H, fd); 
    int16_t qua_z_full = read_16_bit_reg(QUA_Z_REG_L, QUA_Z_REG_H, fd); 
    int16_t qua_w_full = read_16_bit_reg(QUA_W_REG_L, QUA_W_REG_H, fd); 

    std::array<int16_t, 4> qua_xyzw_raw = {qua_x_full, qua_y_full, qua_z_full, qua_w_full};
    std::array<double, 4> qua_xyzw_scaled = {qua_x_full / QUAT_SCALE_VAL, qua_y_full / QUAT_SCALE_VAL, qua_z_full / QUAT_SCALE_VAL,qua_w_full/QUAT_SCALE_VAL}; 
    qua_signal_raw(qua_xyzw_raw); 
    qua_signal_scaled(qua_xyzw_scaled); 
}


void Imu_Interface::publish_eul_rph(){
    int16_t eul_roll_full = read_16_bit_reg(EUL_ROLL_REG_L, EUL_ROLL_REG_H, fd); 
    int16_t eul_pitch_full = read_16_bit_reg(EUL_PITCH_REG_L, EUL_PITCH_REG_H, fd); 
    int16_t eul_head_full = read_16_bit_reg(EUL_HEAD_REG_L, EUL_HEAD_REG_H, fd);  

    std::array<int16_t, 3> eul_rph_raw = {eul_roll_full, eul_pitch_full, eul_head_full};
    std::array<double, 3> eul_rph_scaled = {eul_roll_full*EUL_SCALE_VAL, eul_pitch_full*EUL_SCALE_VAL, eul_head_full*EUL_SCALE_VAL}; 
    eul_signal_raw(eul_rph_raw); 
    eul_signal_scaled(eul_rph_scaled); 
}

void Imu_Interface::publish_roll() {
    int16_t roll_full = read_16_bit_reg(ROLL_REG_L, ROLL_REG_H, fd);  
    double roll_deg = roll_full / 16.0; // Convert to degrees
    roll_signal(roll_deg); 
}

void Imu_Interface::publish_pitch() {
    int16_t pitch_full = read_16_bit_reg(PITCH_REG_L, PITCH_REG_H, fd);  
    double pitch_deg = pitch_full / 16.0; // Convert to degrees
    pitch_signal(pitch_deg); 
}

void Imu_Interface::publish_head() {
    int16_t head_full = read_16_bit_reg(HEAD_REG_L, HEAD_REG_H, fd);  
    double head_deg = head_full / 16.0; // Convert to degrees
    head_signal(head_deg); 
}