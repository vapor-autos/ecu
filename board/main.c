#include "can.h"
#include "castle.h"
#include "config.h"
#include "defines.h"
#include "early_init.h"
#include "ecu.h"
#include "setup.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_pcd.h"
#include "version.h"
#include <stdbool.h>
#include <stdint.h>

// This runs before main application
void __initialize_hardware_early(void) {
    early_initialization();
}

//------------------------------------------------------------------------
// Global
//------------------------------------------------------------------------
extern ecu_t ecu;
castle_t castle_data;
volatile uint16_t throttle_val = THROTTLE_NEUTRAL;
volatile uint32_t steering_val = STEERING_CENTER;
volatile uint16_t speed = 0;
volatile int16_t steering_angle = 0;

volatile uint32_t can_isr_count = 0;
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_send_count = 0;
volatile uint32_t can_overflow_count = 0;
volatile uint32_t can_last_error = 0;
volatile uint32_t can_error_count = 0;
volatile bool cruise_enabled;
volatile uint32_t last_can_rx_time = 0;
uint8_t i2c_fail_streak = 0;
volatile bool i2c_connection_bad = false;

// CAN RX circular buffer
CAN_RxMessage CAN_RxBuffer[CAN_RX_BUFFER_SIZE];
volatile uint32_t CAN_RxBufferHead = 0;
volatile uint32_t CAN_RxBufferTail = 0;

//------------------------------------------------------------------------
// Local
//------------------------------------------------------------------------
static uint32_t tick_prev = 0U;
static uint32_t main_loop_1Hz = 0U;
static uint32_t main_loop_1Hz_runtime = 0U;
static uint32_t main_loop_20Hz = 0U;
static uint32_t main_loop_20Hz_runtime = 0U;
static uint32_t main_loop_50Hz = 0U;
static uint32_t main_loop_50Hz_runtime = 0U;
static uint32_t main_loop_100Hz = 0U;
static uint32_t main_loop_100Hz_runtime = 0U;

static inline void set_status_led(bool i2c_bad, bool can_good) {
    // RGB LEDs are active-low: RESET=ON, SET=OFF.
    HAL_GPIO_WritePin(GPIOC, ecu.led_red_pin | ecu.led_green_pin | ecu.led_blue_pin, GPIO_PIN_SET);

    if (i2c_bad) {
        HAL_GPIO_WritePin(GPIOC, ecu.led_red_pin, GPIO_PIN_RESET);
    } else if (can_good) {
        HAL_GPIO_WritePin(GPIOC, ecu.led_blue_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, ecu.led_green_pin, GPIO_PIN_RESET);
    }
}

int main(void) {
    HAL_Init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    // Initialize system clock
    __HAL_RCC_DMA2_CLK_DISABLE();
    SystemClock_Config();
    // Initialize all hardware components
    ecu_init();
    MX_GPIO_Clocks_Init();
    MX_TIM_Init();
    MX_GPIO_Common_Init();
    MX_I2C_Init();
    MX_UART_Init();
    MX_CAN_DEVICE_Init();

    while (1) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - tick_prev) >= 1U) { // 1kHz
            tick_prev = now;
            uint32_t loop_now = now;

            bool can_good = (uint32_t)(loop_now - last_can_rx_time) <= CAN_RX_TIMEOUT_MS;
            set_status_led(i2c_connection_bad, can_good);

            // 1hz loop
            if ((uint32_t)(loop_now - main_loop_1Hz) >= 1000U) {
                uint32_t start = loop_now;
                uart_printf("Timestamp(1hz): %d\n", start);
                uart_printf("CAN ISR count: %lu\n", can_isr_count);
                uart_printf("CAN RX count: %lu\n", can_rx_count);
                uart_printf("CAN send count: %lu\n", can_send_count);
                uart_printf("CAN error count: %lu\n", can_error_count);
                uart_printf("CAN last error: 0x%08lX\n", can_last_error);
                uart_printf("Steering value: %d\n", steering_val);
                uart_printf("Steering angle: %d\n", steering_angle);
                uart_printf("Speed: %d\n", speed);
                uart_printf("Throttle value: %d\n", throttle_val);
                main_loop_1Hz_runtime = HAL_GetTick() - start;
                main_loop_1Hz = start;
                loop_now = HAL_GetTick();
            }
            // 20hz loop
            if ((uint32_t)(loop_now - main_loop_20Hz) >= 50U) {
                uint32_t start = loop_now;
                speed = read_filtered_speed();
                main_loop_20Hz_runtime = HAL_GetTick() - start;
                main_loop_20Hz = start;
                loop_now = HAL_GetTick();
            }
            // 50hz loop
            if ((uint32_t)(loop_now - main_loop_50Hz) >= 20U) {
                uint32_t start = loop_now;

                // Safety decay on CAN timeout
                if ((uint32_t)(loop_now - last_can_rx_time) > CAN_RX_TIMEOUT_MS) {
                    // Throttle decay toward neutral
                    uint16_t throttle_local = throttle_val;
                    if (throttle_local > THROTTLE_NEUTRAL) {
                        throttle_local = (throttle_local - THROTTLE_DECAY_STEP < THROTTLE_NEUTRAL)
                                             ? THROTTLE_NEUTRAL
                                             : throttle_local - THROTTLE_DECAY_STEP;
                    } else if (throttle_local < THROTTLE_NEUTRAL) {
                        throttle_local = (throttle_local + THROTTLE_DECAY_STEP > THROTTLE_NEUTRAL)
                                             ? THROTTLE_NEUTRAL
                                             : throttle_local + THROTTLE_DECAY_STEP;
                    }
                    throttle_val = throttle_local;

                    // Steering decay toward center
                    uint32_t steering_local = steering_val;
                    if (steering_local > STEERING_CENTER) {
                        steering_local = (steering_local - STEERING_DECAY_STEP < STEERING_CENTER)
                                             ? STEERING_CENTER
                                             : steering_local - STEERING_DECAY_STEP;
                    } else if (steering_local < STEERING_CENTER) {
                        steering_local = (steering_local + STEERING_DECAY_STEP > STEERING_CENTER)
                                             ? STEERING_CENTER
                                             : steering_local + STEERING_DECAY_STEP;
                    }
                    steering_val = steering_local;
                }

                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, steering_val);
                write_i2c_reg(&hi2c3, WriteThrottle, throttle_val);
                main_loop_50Hz_runtime = HAL_GetTick() - start;
                main_loop_50Hz = start;
                loop_now = HAL_GetTick();
            }
            // 100hz loop
            if ((uint32_t)(loop_now - main_loop_100Hz) >= 10U) {
                uint32_t start = loop_now;
                send_can_msg();
                main_loop_100Hz_runtime = HAL_GetTick() - start;
                main_loop_100Hz = start;
            }
            process_can_msgs();
        }
    }
}
