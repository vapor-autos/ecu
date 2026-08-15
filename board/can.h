#ifndef CAN_H
#define CAN_H

#include "castle.h"
#include "config.h"
#include "defines.h"
#include "stdout.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include <stdbool.h>
#include <string.h>

// Forward declarations
extern CAN_HandleTypeDef hcan2;
extern I2C_HandleTypeDef hi2c3;
extern ecu_t ecu;

#define CAN_RX_BUFFER_SIZE 32

#define CAN_RX_TIMEOUT_MS 500
#define THROTTLE_DECAY_STEP 100
#define STEERING_DECAY_STEP 20
// Steering trim in centidegrees. Positive values make a zero-degree command
// apply left trim at the servo while CAN feedback still echoes the command.
#define STEERING_ANGLE_OFFSET 820

// CAN message IDs
#define CAN_ID_STEERING 0x202
#define CAN_ID_THROTTLE 0x203
#define CAN_ID_HEADLIGHT 0x204
#define CAN_ID_CRUISE_ENABLE 0x205
#define CAN_ID_STEERING_ANGLE 0x208
#define CAN_ID_SPEED 0x209

extern volatile uint16_t throttle_val;
extern volatile uint32_t steering_val;
extern volatile int16_t steering_angle;
extern volatile uint16_t speed;
extern volatile uint32_t can_isr_count;
extern volatile uint32_t can_rx_count;
extern volatile uint32_t can_send_count;
extern volatile uint32_t can_overflow_count;
extern volatile uint32_t can_last_error;
extern volatile uint32_t can_error_count;
extern volatile bool cruise_enabled;
extern volatile uint32_t last_can_rx_time;

typedef struct {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
} CAN_RxMessage;

extern CAN_RxMessage CAN_RxBuffer[CAN_RX_BUFFER_SIZE];
extern volatile uint32_t CAN_RxBufferHead;
extern volatile uint32_t CAN_RxBufferTail;

/*
Interrupt Handlers
*/

// CAN RX FIFO interrupt callback - fills circular buffer for main loop processing
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        can_isr_count++;
        uint32_t nextHead = (CAN_RxBufferHead + 1) % CAN_RX_BUFFER_SIZE;
        if (nextHead != CAN_RxBufferTail) {
            CAN_RxBuffer[CAN_RxBufferHead].RxHeader = RxHeader;
            memcpy(CAN_RxBuffer[CAN_RxBufferHead].RxData, RxData, sizeof(RxData));
            __DMB(); // Ensure RX data is visible before updating head.
            CAN_RxBufferHead = nextHead;
        } else {
            can_overflow_count++;
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    uint32_t error = HAL_CAN_GetError(hcan);
    can_last_error = error;
    can_error_count++;

    __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FF0 | CAN_FLAG_FOV0);

    if (error & HAL_CAN_ERROR_BOF) {
        HAL_CAN_Stop(hcan);
        if (HAL_CAN_Start(hcan) != HAL_OK) {
            uart_printf("Failed to restart CAN after Bus-Off\n");
        } else {
            uart_printf("CAN Bus-Off recovery successful\n");
        }
    } else {
        // Reset the CAN peripheral for other errors
        HAL_CAN_ResetError(hcan);
    }
}

void CAN2_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan2);
}

static uint8_t msg_counter = 0;

// Send CAN messages in round-robin fashion to avoid filling all TX mailboxes
static inline void send_can_msg(void) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    HAL_StatusTypeDef status;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    switch (msg_counter % 4) {
    case 0: {
        uint8_t cruise_data = cruise_enabled ? 1 : 0;
        TxHeader.StdId = CAN_ID_CRUISE_ENABLE;
        TxHeader.DLC = 1;

        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0) {
            can_send_count++;
            status = HAL_CAN_AddTxMessage(&hcan2, &TxHeader, &cruise_data, &TxMailbox);
            if (status != HAL_OK) {
                uart_printf("CAN send failed for cruise enable, status=%d\n", status);
            } else {
                msg_counter++;
            }
        }
        break;
    }
    case 1:
        // Intentionally empty - spreads out message timing
        msg_counter++;
        break;
    case 2: {
        uint16_t safe_speed = speed;
        uint8_t speed_data[2];
        speed_data[0] = safe_speed & 0xFF;
        speed_data[1] = (safe_speed >> 8) & 0xFF;

        TxHeader.StdId = CAN_ID_SPEED;
        TxHeader.DLC = 2;
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0) {
            can_send_count++;
            status = HAL_CAN_AddTxMessage(&hcan2, &TxHeader, speed_data, &TxMailbox);
            if (status != HAL_OK) {
                uart_printf("CAN send failed for speed, status=%d\n", status);
            } else {
                msg_counter++;
            }
        }
        break;
    }
    case 3: {
        int16_t steer = steering_angle;
        uint8_t steer_data[2];
        steer_data[0] = steer & 0xFF;
        steer_data[1] = (steer >> 8) & 0xFF;
        TxHeader.StdId = CAN_ID_STEERING_ANGLE;
        TxHeader.DLC = 2;
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0) {
            can_send_count++;
            status = HAL_CAN_AddTxMessage(&hcan2, &TxHeader, steer_data, &TxMailbox);
            if (status != HAL_OK) {
                uart_printf("CAN send failed for steering angle, status=%d\n", status);
            } else {
                msg_counter++;
            }
        }
        break;
    }
    default:
        break;
    }
}

// Process received CAN message and update control state
static inline void handle_can_msg(CAN_RxMessage msg) {
    can_rx_count++;
    if (msg.RxHeader.IDE != CAN_ID_STD || msg.RxHeader.RTR != CAN_RTR_DATA) {
        return;
    }
    switch (msg.RxHeader.StdId) {
    case CAN_ID_THROTTLE:
        if (msg.RxHeader.DLC == 2) {
            int16_t throttle_16 = (int16_t)(msg.RxData[0] | (msg.RxData[1] << 8));
            throttle_val = convert_throttle(throttle_16);
            last_can_rx_time = HAL_GetTick();
        } else {
            uart_printf("Invalid throttle message\n");
        }
        break;
    case CAN_ID_STEERING:
        if (msg.RxHeader.DLC >= 2) {
            int16_t steer_cmd = (int16_t)(msg.RxData[0] | (msg.RxData[1] << 8));
            int16_t applied_steer = steer_cmd - STEERING_ANGLE_OFFSET;
            steering_angle = steer_cmd;
            steering_val = convert_steering(applied_steer);
            last_can_rx_time = HAL_GetTick();
        } else {
            uart_printf("Invalid steer message DLC\n");
        }
        break;
    case CAN_ID_HEADLIGHT: {
        if (msg.RxHeader.DLC >= 1) {
            int8_t headlight_enable = (int8_t)msg.RxData[0];
            if (headlight_enable == 0) {
                HAL_GPIO_WritePin(GPIOA, ecu.rear_headlights_pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, ecu.front_headlights_pin, GPIO_PIN_RESET);
            } else if (headlight_enable == 1) {
                HAL_GPIO_WritePin(GPIOA, ecu.rear_headlights_pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, ecu.front_headlights_pin, GPIO_PIN_SET);
            } else {
                uart_printf("Invalid headlight message\n");
            }
            last_can_rx_time = HAL_GetTick();
        } else {
            uart_printf("Invalid headlight message DLC\n");
        }
        break;
    }
    case CAN_ID_CRUISE_ENABLE:
        if (msg.RxHeader.DLC >= 1) {
            uint8_t cruise_enable = msg.RxData[0];
            if (cruise_enable == 0) {
                cruise_enabled = false;
                uart_printf("Cruise control disabled\n");
            } else if (cruise_enable == 1) {
                cruise_enabled = true;
                uart_printf("Cruise control enabled\n");
            } else {
                uart_printf("Invalid cruise enable value: %d\n", cruise_enable);
            }
            last_can_rx_time = HAL_GetTick();
        } else {
            uart_printf("Invalid cruise enable message DLC\n");
        }
        break;
    default:
        break;
    }
}

// Process all pending CAN messages from circular buffer
static inline void process_can_msgs(void) {
    while (CAN_RxBufferHead != CAN_RxBufferTail) {
        __DMB(); // Ensure we see buffer data written before head update
        CAN_RxMessage msg = CAN_RxBuffer[CAN_RxBufferTail];
        CAN_RxBufferTail = (CAN_RxBufferTail + 1) % CAN_RX_BUFFER_SIZE;
        handle_can_msg(msg);
    }
}

#endif
