#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>
#include "config.h" // Ensures we have access to MPU_ACCEL_FSR and MPU_GYRO_FSR

// --- IMU Physical Offsets (in meters) ---
#define IMU_OFFSET_X  0.2032f 
#define IMU_OFFSET_Y  0.0000f 
#define IMU_OFFSET_Z  0.0254f 

// --- Compile-Time Sensor Scaling Macros ---
#define GYRO_LSB_PER_DPS ( (MPU_GYRO_FSR == 250) ? 131.0f : \
                           (MPU_GYRO_FSR == 500) ? 65.5f : \
                           (MPU_GYRO_FSR == 1000) ? 32.8f : 16.4f )

#define ACCEL_LSB_PER_G  ( (MPU_ACCEL_FSR == 2) ? 16384.0f : \
                           (MPU_ACCEL_FSR == 4) ? 8192.0f : \
                           (MPU_ACCEL_FSR == 8) ? 4096.0f : 2048.0f )

#define GYRO_RAW_TO_RADS  (0.01745329251f / GYRO_LSB_PER_DPS) // (pi/180) / LSB
#define ACCEL_RAW_TO_MS2  (9.80665f / ACCEL_LSB_PER_G)        // 9.81 / LSB

// Mahony Filter Gains
#define TWO_KP 1.0f 
#define TWO_KI 0.0f 

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float pitch_rate; // Mapped directly to gyro, immune to loop jitter
} IMU_State_t;

extern IMU_State_t imu_state;

void imu_filter_init(void);
void imu_filter_update(float ax, float ay, float az, float gx, float gy, float gz, float dt);

#endif