#include "../include/umrt_imu_interface/imu.h"



void lin_acc_xyz_handler(std::array<int16_t, 3> lin_acc_xyz_raw) {
    BOOST_LOG_TRIVIAL(info) << "Lin Acc raw: x=" << lin_acc_xyz_raw[0] << ", y=" << lin_acc_xyz_raw[1] << ", z=" << lin_acc_xyz_raw[2];
}

void lin_acc_xyz_scaled_handler(std::array<double, 3> lin_acc_xyz_scaled) {
    BOOST_LOG_TRIVIAL(info) << "Lin Acc scaled (m/s²): x=" << lin_acc_xyz_scaled[0] << ", y=" << lin_acc_xyz_scaled[1] << ", z=" << lin_acc_xyz_scaled[2];
}

void eul_rph_handler(std::array<int16_t, 3> eul_rph_raw) {
    BOOST_LOG_TRIVIAL(info) << "Euler raw: roll=" << eul_rph_raw[0] << ", pitch=" << eul_rph_raw[1] << ", head=" << eul_rph_raw[2];
}

void eul_rph_scaled_handler(std::array<double, 3> eul_rph_scaled) {
    BOOST_LOG_TRIVIAL(info) << "Euler scaled: roll=" << eul_rph_scaled[0] << ", pitch=" << eul_rph_scaled[1] << ", head=" << eul_rph_scaled[2];
}
void qua_xyzw_handler(std::array<int16_t, 4> qua_xyzw_raw) {
    BOOST_LOG_TRIVIAL(info) << "Quaternion raw x=" << qua_xyzw_raw[0] << ", y=" << qua_xyzw_raw[1] << ", z=" << qua_xyzw_raw[2] <<", w="<<qua_xyzw_raw[4];
}

void qua_xyzw_scaled_handler(std::array<double, 4> qua_xyzw_scaled) {
    BOOST_LOG_TRIVIAL(info) << "Quaternion scaled x=" << qua_xyzw_scaled[0] << ", y=" << qua_xyzw_scaled[1] << ", z=" << qua_xyzw_scaled[2] <<", w="<<qua_xyzw_scaled[4];
}

int main(int argc, char* argv[]) {
    char* address = "/dev/i2c-1"; 
    uint8_t slave_addr = 0x28;  
    if (argc > 1) {
        address = argv[1]; 
    }
    Imu_Interface imu_interface(address, slave_addr);
    imu_interface.lin_acc_signal_raw.connect([](std::array<int16_t, 3> lin_acc_xyz_raw) { lin_acc_xyz_handler(lin_acc_xyz_raw); });
    imu_interface.lin_acc_signal_scaled.connect([](std::array<double, 3> lin_acc_xyz_scaled) { lin_acc_xyz_scaled_handler(lin_acc_xyz_scaled); });

    imu_interface.eul_signal_raw.connect([](std::array<int16_t, 3> eul_rph_raw) { eul_rph_handler(eul_rph_raw); });
    imu_interface.eul_signal_scaled.connect([](std::array<double, 3> eul_rph_scaled) { eul_rph_scaled_handler(eul_rph_scaled); });

    imu_interface.qua_signal_raw.connect([](std::array<int16_t, 4> qua_xyzw_raw) { qua_xyzw_handler(qua_xyzw_raw); });
    imu_interface.qua_signal_scaled.connect([](std::array<double, 4> qua_xyzw_scaled) { qua_xyzw_scaled_handler(qua_xyzw_scaled); });

    imu_interface.initialize_read_loop(1000000000); 
    
    return 0; 
}