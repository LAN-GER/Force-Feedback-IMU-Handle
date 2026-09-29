#pragma once

/* 板级引脚：自研 ESP32-C3FH4 最小系统（力反馈 IMU 手柄）
 * 必须与 hardware/pinmap.md 保持一致。
 *
 * SPI 总线沿用 CAN_Magnetic_Encoder 已验证线序：
 *   SCK=GPIO8, MISO=GPIO6, MOSI=GPIO10（禁止 MISO 接 GPIO9/BOOT）
 * BMI088 双片选：CS_ACC=GPIO5, CS_GYRO=GPIO7
 */

#include "driver/gpio.h"
#include "driver/ledc.h"

/* BMI088 SPI */
#define BSP_PIN_IMU_SCK     GPIO_NUM_8
#define BSP_PIN_IMU_MISO    GPIO_NUM_6
#define BSP_PIN_IMU_MOSI    GPIO_NUM_10
#define BSP_PIN_IMU_CS_ACC  GPIO_NUM_5
#define BSP_PIN_IMU_CS_GYRO GPIO_NUM_7
/* YJL3400A 栅极：低侧开关驱动贴片 100Ω 加热电阻，抑制 BMI088 温漂 */
#define BSP_PIN_IMU_HEATER  GPIO_NUM_21

/* DRV8833 + N20 */
#define BSP_PIN_MOTOR_IN1   GPIO_NUM_2
#define BSP_PIN_MOTOR_IN2   GPIO_NUM_3
#define BSP_PIN_MOTOR_NSLEEP GPIO_NUM_20
#define BSP_PIN_ENC_A       GPIO_NUM_0
#define BSP_PIN_ENC_B       GPIO_NUM_1

#define BSP_PIN_WS2812      GPIO_NUM_4
#define BSP_WS2812_COUNT    1

/* 用户键与 BOOT 共用 GPIO9：按下为低；上电时勿按住，否则进下载模式 */
#define BSP_PIN_USER_BTN    GPIO_NUM_9

#define BSP_PIN_USER_LED    (-1)
#define BSP_HAS_USER_LED    0

#define BSP_SPI_HOST        SPI2_HOST
#define BSP_SPI_HZ          8000000

#define BSP_MOTOR_PWM_HZ    20000
#define BSP_MOTOR_PWM_RES   LEDC_TIMER_10_BIT
#define BSP_MOTOR_PWM_MAX   1023

#define BSP_HEATER_PWM_HZ   1000
#define BSP_HEATER_PWM_RES  LEDC_TIMER_10_BIT
#define BSP_HEATER_PWM_MAX  1023
