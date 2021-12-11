#ifndef PINS_BTTSKRMINIE3V2_H
#define PINS_BTTSKRMINIE3V2_H

#include "stm32f1xx_hal.h"

// https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-SKR-mini-E3/master/hardware/BTT%20SKR%20MINI%20E3%20V2.0/Hardware/BTT%20SKR%20MINI%20E3%20V2.0SCHpdf.PDF
// https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3/blob/3419f21a799b2598ec58ea037f4093ddcbbc318f/firmware/V2.0/Marlin-2.0.8.2.x-SKR-mini-E3-V2.0/Marlin/src/pins/stm32f1/pins_BTT_SKR_MINI_E3_common.h

// Release PB3/PB4 (E0 STP/DIR) from JTAG pins
// #define DISABLE_JTAG

//
// Steppers
//
#define X_ENABLE_PIN GPIO_PIN_14
#define X_STEP_PIN GPIO_PIN_13
#define X_DIR_PIN GPIO_PIN_12
#define X_STOP_PIN GPIO_PIN_0

#define X_ENABLE_PORT GPIOB
#define X_STEP_PORT GPIOB
#define X_DIR_PORT GPIOB
#define X_STOP_PORT GPIOC


#define Y_ENABLE_PIN PB11
#define Y_STEP_PIN PB10
#define Y_DIR_PIN PB2
#define Y_STOP_PIN PC1

#define Z_ENABLE_PIN PB1
#define Z_STEP_PIN PB0
#define Z_DIR_PIN PC5
#define Z_STOP_PIN PC2

#define E0_ENABLE_PIN PD2
#define E0_STEP_PIN PB3
#define E0_DIR_PIN PB4
#define E0_STOP_PIN PC15

/**
   * TMC220x stepper drivers
   * Hardware serial communication ports
   */
#define RX4_PIN PC11
#define TX4_PIN PC10
#define X_HARDWARE_SERIAL MSerial4
#define Y_HARDWARE_SERIAL MSerial4
#define Z_HARDWARE_SERIAL MSerial4
#define E0_HARDWARE_SERIAL MSerial4

// Default TMC slave addresses
#define X_SLAVE_ADDRESS 0
#define Y_SLAVE_ADDRESS 2
#define Z_SLAVE_ADDRESS 1
#define E0_SLAVE_ADDRESS 3

// I2C
// #define I2C_EEPROM
// #define SOFT_I2C_EEPROM // Force the use of Software I2C
// #define I2C_SCL_PIN PB6
// #define I2C_SDA_PIN PB7
// #define MARLIN_EEPROM_SIZE 0x1000 // 4KB

#endif // PINS_BTTSKRMINIE3V2_H