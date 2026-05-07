#include "imu_filter.h"
#include <math.h>

IMU_State_t imu_state;

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
static float gx_prev = 0.0f, gy_prev = 0.0f, gz_prev = 0.0f;

// Compiler-safe Fast Inverse Square Root
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    
    // Using a union safely bypasses strict-aliasing rules in C99
    union {
        float f;
        int32_t i; 
    } conv;
    
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    
    return conv.f;
}

void imu_filter_init(void) {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    gx_prev = 0.0f; gy_prev = 0.0f; gz_prev = 0.0f;
}

// 1. VESC's Kinematic Offset Math
static void correct_lever_arm(float *ax, float *ay, float *az, 
                              float gx, float gy, float gz, float dt) {
    
    // Calculate angular acceleration (derivative of gyro)
    float dwx = (gx - gx_prev) / dt;
    float dwy = (gy - gy_prev) / dt;
    float dwz = (gz - gz_prev) / dt;

    // Save current gyro for next loop
    gx_prev = gx;
    gy_prev = gy;
    gz_prev = gz;

    // Term 1: Angular Acceleration cross Offset (dw x r)
    float t1_x = dwy * IMU_OFFSET_Z - dwz * IMU_OFFSET_Y;
    float t1_y = dwz * IMU_OFFSET_X - dwx * IMU_OFFSET_Z;
    float t1_z = dwx * IMU_OFFSET_Y - dwy * IMU_OFFSET_X;

    // Term 2: Centripetal Acceleration: w x (w x r)
    // First find v = w x r
    float v_x = gy * IMU_OFFSET_Z - gz * IMU_OFFSET_Y;
    float v_y = gz * IMU_OFFSET_X - gx * IMU_OFFSET_Z;
    float v_z = gx * IMU_OFFSET_Y - gy * IMU_OFFSET_X;
    
    // Now w x v
    float t2_x = gy * v_z - gz * v_y;
    float t2_y = gz * v_x - gx * v_z;
    float t2_z = gx * v_y - gy * v_x;

    // Subtract lever arm effects from raw accelerometer readings
    *ax = *ax - (t1_x + t2_x);
    *ay = *ay - (t1_y + t2_y);
    *az = *az - (t1_z + t2_z);
}

// 2. The Mahony AHRS Filter
static void mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // ---> USE FAST MATH HERE <---
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Apply proportional and integral feedback
        if(TWO_KI > 0.0f) {
            integralFBx += TWO_KI * halfex * dt;
            integralFBy += TWO_KI * halfey * dt;
            integralFBz += TWO_KI * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += TWO_KP * halfex;
        gy += TWO_KP * halfey;
        gz += TWO_KP * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// 3. The Master Update Wrapper
void imu_filter_update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    
    // 1. Correct the accelerometer data based on lever arm
    correct_lever_arm(&ax, &ay, &az, gx, gy, gz, dt);

    // 2. Pass corrected accel and raw gyro into the filter
    mahony_update(ax, ay, az, gx, gy, gz, dt);

    // 3. Extract Euler angles (assuming standard aerospace sequence)
    imu_state.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    
    // Safety clamp for pitch calculation
    float pitch_val = 2.0f * (q0 * q2 - q3 * q1);
    if (pitch_val > 1.0f) pitch_val = 1.0f;
    if (pitch_val < -1.0f) pitch_val = -1.0f;
    imu_state.pitch = asinf(pitch_val);
    
    imu_state.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    // Expose raw gyro pitch rate for your LQR derivative term
    // (Adjust gx/gy/gz mapping based on how your IMU is mounted)
    imu_state.pitch_rate = gy; 
}