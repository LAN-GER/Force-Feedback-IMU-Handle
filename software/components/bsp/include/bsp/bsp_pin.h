#pragma once

/* 板级引脚：自研 ESP32-C3FH4（DRV8876 电流检测版）
 * 必须与 hardware/pinmap.md 保持一致。
 *
 * SPI：SCK=GPIO8, MISO=GPIO6, MOSI=GPIO10（禁止 MISO 接 GPIO9/BOOT）
 * BMI088：CS_ACC=GPIO5, CS_GYRO=GPIO7
 * DRV8876 PH/EN：EN=GPIO2(PWM), PH=GPIO3(DIR), nSLEEP=GPIO20, IPROPI=GPIO4(ADC)
 */

#include "driver/gpio.h"
#include "driver/ledc.h"

/* BMI088 SPI */
#define BSP_PIN_IMU_SCK     GPIO_NUM_8
#define BSP_PIN_IMU_MISO    GPIO_NUM_6
#define BSP_PIN_IMU_MOSI    GPIO_NUM_10
#define BSP_PIN_IMU_CS_ACC  GPIO_NUM_5
#define BSP_PIN_IMU_CS_GYRO GPIO_NUM_7
/* YJL3400A 栅极：低侧开关驱动贴片 100Ω 加热电阻 */
#define BSP_PIN_IMU_HEATER  GPIO_NUM_21

/* DRV8876 + N20（PMODE=GND → PH/EN） */
#define BSP_PIN_MOTOR_EN    GPIO_NUM_2   /* EN/IN1：LEDC PWM */
#define BSP_PIN_MOTOR_PH    GPIO_NUM_3   /* PH/IN2：方向 */
#define BSP_PIN_MOTOR_NSLEEP GPIO_NUM_20
#define BSP_PIN_MOTOR_IPROPI GPIO_NUM_4  /* ADC1_CH4 电流检测 */
#define BSP_PIN_ENC_A       GPIO_NUM_0
#define BSP_PIN_ENC_B       GPIO_NUM_1

/* 兼容旧名（双 PWM 时代）；新板只用 EN */
#define BSP_PIN_MOTOR_IN1   BSP_PIN_MOTOR_EN
#define BSP_PIN_MOTOR_IN2   BSP_PIN_MOTOR_PH

/* 本板无 WS2812 */
#define BSP_HAS_WS2812      0

/* 用户键与 BOOT 共用 GPIO9：按下为低；上电时勿按住 */
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

/* DRV8876 IPROPI：R_IPROPI=2.2kΩ，A_IPROPI=1000µA/A → I = V / 2.2 */
#define BSP_IPROPI_R_OHM    2200.0f
#define BSP_IPROPI_A        0.001f
