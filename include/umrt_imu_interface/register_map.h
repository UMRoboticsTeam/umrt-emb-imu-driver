#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H 

const uint8_t LIN_ACC_X_REG_H = 0x29; 
const uint8_t LIN_ACC_X_REG_L = 0x28; 
const uint8_t LIN_ACC_Y_REG_H = 0x2B; 
const uint8_t LIN_ACC_Y_REG_L = 0x2A; 
const uint8_t LIN_ACC_Z_REG_H = 0x2D; 
const uint8_t LIN_ACC_Z_REG_L = 0x2C; 
const uint8_t PITCH_REG_H = 0x1F;
const uint8_t PITCH_REG_L = 0x1E;
const uint8_t ROLL_REG_H = 0x1D; 
const uint8_t ROLL_REG_L = 0x1C;
const uint8_t HEAD_REG_H = 0x1B; 
const uint8_t HEAD_REG_L = 0x1A; 
const uint8_t GYR_X_REG_H = 0x15;
const uint8_t GYR_X_REG_L = 0x14; 
const uint8_t GYR_Y_REG_H = 0x17; 
const uint8_t GYR_Y_REG_L = 0x16; 
const uint8_t GYR_Z_REG_H = 0x19;
const uint8_t GYR_Z_REG_L = 0x18; 
const uint8_t MAG_X_REG_H = 0x0F;  
const uint8_t MAG_X_REG_L = 0x0E;
const uint8_t MAG_Y_REG_H = 0x11; 
const uint8_t MAG_Y_REG_L = 0x10; 
const uint8_t MAG_Z_REG_H = 0x13; 
const uint8_t MAG_Z_REG_L = 0x12; 
const uint8_t ACC_X_REG_H = 0x09; 
const uint8_t ACC_X_REG_L = 0x08;
const uint8_t ACC_Y_REG_H = 0x0B; 
const uint8_t ACC_Y_REG_L = 0x0A; 
const uint8_t ACC_Z_REG_H = 0x0D; 
const uint8_t ACC_Z_REG_L = 0x0C; 
const uint8_t OPR_MODE_REG = 0x3D; 
const uint8_t OPR_MODE_VAL = 0x08; // IMU mode for debugging (was 0b00001100 for NDOF)
const uint8_t CHIP_ID_REG = 0x00; // Chip ID register
const uint8_t SYS_STATUS_REG = 0x39; // System status
const uint8_t CALIB_STAT_REG = 0x35; // Calibration status
const uint8_t SYS_TRIGGER_REG = 0x3F; // System trigger
const uint8_t QUA_X_REG_L = 0x22; 
const uint8_t QUA_X_REG_H = 0x23; 
const uint8_t QUA_Y_REG_L = 0x24; 
const uint8_t QUA_Y_REG_H = 0x25;
const uint8_t QUA_Z_REG_L = 0x26; 
const uint8_t QUA_Z_REG_H = 0x27;
const uint8_t QUA_W_REG_L = 0x20; 
const uint8_t QUA_W_REG_H = 0x21;
const uint8_t EUL_PITCH_REG_H = 0x1F; 
const uint8_t EUL_PITCH_REG_L = 0x1E; 
const uint8_t EUL_ROLL_REG_H = 0x1D; 
const uint8_t EUL_ROLL_REG_L = 0x1C; 
const uint8_t EUL_HEAD_REG_H = 0x1B; 
const uint8_t EUL_HEAD_REG_L = 0x1A; 



#endif