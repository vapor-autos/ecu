#ifndef CASTLE_H
#define CASTLE_H

#include "stdout.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Forward declarations
extern I2C_HandleTypeDef hi2c3;

#define STEERING_CENTER 6000   // pulse ticks, 4800-7200 range
#define THROTTLE_NEUTRAL 32767 // 16-bit midpoint for Castle ESC

#define STEERING_INPUT_MIN -18000
#define STEERING_INPUT_MAX 18000
#define STEERING_OUTPUT_DELTA 1200

#define THROTTLE_INPUT_MIN -10000
#define THROTTLE_INPUT_MAX 10000
#define THROTTLE_INPUT_SPAN (THROTTLE_INPUT_MAX - THROTTLE_INPUT_MIN)
#define THROTTLE_NEUTRAL_RANGE 2000

#define CASTLE_I2C_ADDR 0x08
#define CASTLE_I2C_TIMEOUT_MS 10

// 2.85 internal gear ratio, 21t pinion, 57t spur gear
#define FINAL_DRIVE_RATIO 7.74f // 57/21 × 2.85
#define WHEEL_CIRC_CM 47.12f    // ~150mm rollout diameter => pi * 15.0cm
#define MOTOR_POLE_PAIRS 2U     // Castle 1415 is 4-pole => 2 pole pairs
#define RPM_SCALE_NUM 20416.66f
#define SCALE_DEN 2042.0f

#define SPEED_MAX_CM_S 4470U                // 100 mph
#define SPEED_STOPPED_CM_S 20U              // Treat as stopped below ~0.45 mph
#define SPEED_MAX_RISE_PER_SAMPLE_CM_S 200U // 20 Hz read loop: max +4 m/s per sample
#define SPEED_INVALID_TIMEOUT_SAMPLES 10U   // Hold last speed for 500ms of bad reads, then report zero

enum ReadRegisterAddresses {
    Voltage = 0,
    Ripple,
    Current,
    Throttle,
    Power,
    Speed,
    Temp,
    BECVolt,
    BECCurrent,
    RawNTC,
    RawLinear,
    LinkLive = 25,
    FailSafe,
    EStop,
    PacketIn,
    PacketOut,
    CheckBad,
    PacketBad
};

enum WriteRegisterAddresses {
    WriteThrottle = 128,
    WriteFailSafe,
    WriteEStop,
    WritePacketIn,
    WritePacketOut,
    WriteCheckBad,
    WritePacketBad
};

typedef struct {
    float voltage;
    float ripple;
    float current;
    float throttle;
    float power;
    float speed;
    float temperature;
    float bec_voltage;
    float bec_current;
    float raw_ntc;
    float raw_linear;
    bool link_live;
    bool fail_safe;
    bool e_stop;
    uint16_t packet_in;
    uint16_t packet_out;
    uint16_t check_bad;
    uint16_t packet_bad;
} castle_t;

extern castle_t castle_data;
extern uint8_t i2c_fail_streak;
extern volatile bool i2c_connection_bad;

static inline void reset_i2c(I2C_HandleTypeDef *hi2c) {
    uart_printf("I2C bus error, resetting peripheral\n");
    HAL_StatusTypeDef deinit_status = HAL_I2C_DeInit(hi2c);
    HAL_StatusTypeDef init_status = HAL_I2C_Init(hi2c);

    if (deinit_status != HAL_OK || init_status != HAL_OK) {
        uart_printf("I2C reset failed: deinit=%d init=%d err=0x%08lx state=%d\n", (int)deinit_status, (int)init_status,
                    HAL_I2C_GetError(hi2c), (int)HAL_I2C_GetState(hi2c));
        i2c_connection_bad = true;
    }
}

static inline void i2c_consider_reset(I2C_HandleTypeDef *hi2c, uint32_t err) {
    // Reset only for repeated bus-level faults or if peripheral is stuck BUSY
    if (i2c_fail_streak >= 5 || HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_BUSY) {
        uart_printf("I2C fail streak: %d, err=0x%08lx, state=%d -> resetting peripheral\n", i2c_fail_streak, err,
                    (int)HAL_I2C_GetState(hi2c));
        reset_i2c(hi2c);
        i2c_fail_streak = 0;
    }
}

static inline uint8_t calculate_checksum(const uint8_t *data, size_t size) {
    uint8_t sum = 0;
    for (size_t i = 0; i < size; ++i) {
        sum += data[i];
    }
    return 0 - sum;
}

static inline bool read_i2c_reg(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint16_t *out_value) {
    if (out_value == NULL) {
        return false;
    }
    uint8_t read_buff[3] = {0};
    uint8_t write_buff[5] = {0};
    write_buff[0] = CASTLE_I2C_ADDR << 1;
    write_buff[1] = reg_addr;
    write_buff[2] = 0;
    write_buff[3] = 0;
    write_buff[4] = calculate_checksum(write_buff, 4);

    if (HAL_I2C_Master_Transmit(hi2c, CASTLE_I2C_ADDR << 1, &write_buff[1], sizeof(write_buff) - 1,
                                CASTLE_I2C_TIMEOUT_MS) != HAL_OK) {
        i2c_fail_streak++;
        i2c_connection_bad = true;
        i2c_consider_reset(hi2c, HAL_I2C_GetError(hi2c));
        return false;
    }

    if (HAL_I2C_Master_Receive(hi2c, CASTLE_I2C_ADDR << 1, read_buff, 3, CASTLE_I2C_TIMEOUT_MS) != HAL_OK) {
        uint32_t err = HAL_I2C_GetError(hi2c);
        if (err != 0) {
            i2c_fail_streak++;
            i2c_connection_bad = true;
            i2c_consider_reset(hi2c, err);
        }
        return false;
    }

    uint8_t expected = calculate_checksum(read_buff, 2);
    if (read_buff[2] != expected) {
        i2c_fail_streak++;
        i2c_connection_bad = true;
        i2c_consider_reset(hi2c, HAL_I2C_GetError(hi2c));
        return false;
    }

    i2c_fail_streak = 0;
    i2c_connection_bad = false;
    *out_value = (uint16_t)((read_buff[0] << 8) | read_buff[1]);
    return true;
}

static inline float castle_parse_float(float value, float scale) {
    return value / SCALE_DEN * scale;
}

static inline uint16_t speed_raw_to_cm_s(uint16_t raw_electrical_rpm_reg) {
    float electrical_rpm = (float)raw_electrical_rpm_reg * RPM_SCALE_NUM / SCALE_DEN;
    // Castle Speed register is electrical RPM; convert using motor pole pairs.
    float mech_rpm = electrical_rpm / (float)MOTOR_POLE_PAIRS;

    // Convert motor RPM to speed in m/s
    float wheel_rpm = (float)mech_rpm / FINAL_DRIVE_RATIO;

    // Convert wheel RPM to speed in m/s
    // wheel_rpm * (WHEEL_CIRC_CM/100 m/rev) * (1 min/60 s)
    // = wheel_rpm * WHEEL_CIRC_CM / (100 * 60)
    // = wheel_rpm * WHEEL_CIRC_CM / 6000
    float speed_m_s = wheel_rpm * WHEEL_CIRC_CM / 6000.0f;

    // Convert to centimeters per second (m/s * 100) for higher resolution
    float speed_cm_s = speed_m_s * 100.0f;

    // Clamp to 0-65535 range and return as uint16_t
    if (speed_cm_s > 65535.0f) {
        return 65535;
    }

    return (uint16_t)(speed_cm_s + 0.5f); // Round to nearest int
}

static inline bool read_speed_cm_s(uint16_t *speed_cm_s) {
    if (speed_cm_s == NULL) {
        return false;
    }

    uint16_t raw_electrical_rpm_reg = 0;
    if (!read_i2c_reg(&hi2c3, Speed, &raw_electrical_rpm_reg)) {
        return false;
    }
    if (raw_electrical_rpm_reg == 0xFFFFU) {
        return false; // Castle sentinel for invalid speed sample
    }

    *speed_cm_s = speed_raw_to_cm_s(raw_electrical_rpm_reg);
    return true;
}

static inline uint16_t speed_filter_update(uint16_t sample_cm_s, bool sample_valid) {
    static uint16_t filtered_cm_s = 0;
    static uint8_t pending_zero_exit_count = 0;
    static uint8_t invalid_count = 0;

    if (!sample_valid) {
        if (invalid_count < UINT8_MAX) {
            invalid_count++;
        }
        pending_zero_exit_count = 0;

        if (invalid_count >= SPEED_INVALID_TIMEOUT_SAMPLES) {
            filtered_cm_s = 0;
        }
        return filtered_cm_s;
    }

    invalid_count = 0;

    if (sample_cm_s > SPEED_MAX_CM_S) {
        pending_zero_exit_count = 0;
        return filtered_cm_s;
    }

    if (filtered_cm_s <= SPEED_STOPPED_CM_S && sample_cm_s > SPEED_STOPPED_CM_S) {
        pending_zero_exit_count++;
        if (pending_zero_exit_count < 2U) {
            return filtered_cm_s;
        }
    } else {
        pending_zero_exit_count = 0;
    }

    if (sample_cm_s > filtered_cm_s + SPEED_MAX_RISE_PER_SAMPLE_CM_S) {
        filtered_cm_s += SPEED_MAX_RISE_PER_SAMPLE_CM_S;
    } else {
        filtered_cm_s = sample_cm_s;
    }

    return filtered_cm_s;
}

static inline uint16_t read_filtered_speed(void) {
    uint16_t sample_cm_s = 0;
    bool sample_valid = read_speed_cm_s(&sample_cm_s);
    return speed_filter_update(sample_cm_s, sample_valid);
}

static inline uint16_t read_speed(void) {
    return read_filtered_speed();
}

static inline uint32_t write_i2c_reg(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint16_t data) {
    uint8_t write_buff[5];
    write_buff[0] = CASTLE_I2C_ADDR << 1;
    write_buff[1] = reg_addr;
    write_buff[2] = (uint8_t)(data >> 8);
    write_buff[3] = (uint8_t)(data & 0xFF);
    write_buff[4] = calculate_checksum(write_buff, 4);

    if (HAL_I2C_Master_Transmit(hi2c, CASTLE_I2C_ADDR << 1, &write_buff[1], sizeof(write_buff) - 1,
                                CASTLE_I2C_TIMEOUT_MS) != HAL_OK) {
        uint32_t err = HAL_I2C_GetError(hi2c);
        i2c_fail_streak++;
        i2c_connection_bad = true;
        i2c_consider_reset(hi2c, err);
        return err;
    }

    i2c_fail_streak = 0;
    i2c_connection_bad = false;
    return 0;
}

/* CONTROLS */

// convert_throttle maps signed throttle input (-10000..10000)
// to Castle throttle register space (0..65535).
// Inputs inside the neutral deadband are snapped to its edge:
// neutral center 32767, deadband 30767..34767.
static inline uint16_t convert_throttle(int16_t throttle) {
    // Clamp input to expected CAN contract range
    if (throttle < THROTTLE_INPUT_MIN) {
        throttle = THROTTLE_INPUT_MIN;
    } else if (throttle > THROTTLE_INPUT_MAX) {
        throttle = THROTTLE_INPUT_MAX;
    }

    // Convert [THROTTLE_INPUT_MIN, THROTTLE_INPUT_MAX] to [0, 65535]
    uint16_t desired_throttle = (uint16_t)(((int32_t)throttle - THROTTLE_INPUT_MIN) * 65535 / THROTTLE_INPUT_SPAN);

    // Snap to edge of neutral zone
    if (desired_throttle >= THROTTLE_NEUTRAL - THROTTLE_NEUTRAL_RANGE &&
        desired_throttle <= THROTTLE_NEUTRAL + THROTTLE_NEUTRAL_RANGE) {
        if (throttle > 0) {
            desired_throttle = THROTTLE_NEUTRAL + THROTTLE_NEUTRAL_RANGE;
        } else if (throttle < 0) {
            desired_throttle = THROTTLE_NEUTRAL - THROTTLE_NEUTRAL_RANGE;
        } else {
            desired_throttle = THROTTLE_NEUTRAL; // throttle == 0: hold exactly at center
        }
    }

    return desired_throttle;
}

// convert_steering maps steering input (-18000..18000) to PWM ticks (4800..7200).
// Steering center is 6000 with +/-1200 output range.
static inline uint32_t convert_steering(int32_t steering) {
    // Clamp input to valid range
    if (steering < STEERING_INPUT_MIN)
        steering = STEERING_INPUT_MIN;
    if (steering > STEERING_INPUT_MAX)
        steering = STEERING_INPUT_MAX;

    // Center: STEERING_CENTER (steering = 0), Range: ±STEERING_OUTPUT_DELTA
    return STEERING_CENTER + ((int32_t)steering * STEERING_OUTPUT_DELTA) / STEERING_INPUT_MAX;
}

#endif
