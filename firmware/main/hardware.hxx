/** \copyright
 * Copyright (c) 2021, Mike Dunston
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file hardware.hxx
 *
 * Hardware representation for the ESP32C3OlcbIO.
 *
 * @author Mike Dunston
 * @date 23 May 2021
 */

#ifndef HARDWARE_HXX_
#define HARDWARE_HXX_

#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <hal/gpio_types.h>

/// GPIO Pin connected to the TWAI (CAN) Transceiver RX pin.
static constexpr gpio_num_t TWAI_RX_PIN_NUM = GPIO_NUM_6;

/// GPIO Pin connected to the TWAI (CAN) Transceiver TX pin.
static constexpr gpio_num_t TWAI_TX_PIN_NUM = GPIO_NUM_7;

/// GPIO Pin used for I2C SDA.
static constexpr gpio_num_t SDA_PIN_NUM = GPIO_NUM_4;

/// GPIO Pin used for I2C SCL.
static constexpr gpio_num_t SCL_PIN_NUM = GPIO_NUM_5;

/// GPIO Pin used for Activity indicator LED.
static constexpr gpio_num_t LED_ACTIVITY_PIN_NUM = GPIO_NUM_0;

/// GPIO Pin used for Bootloader Write indicator LED.
static constexpr gpio_num_t LED_BOOTLOADER_WRITE_PIN_NUM = GPIO_NUM_1;

/// GPIO Pin used for Bootloader indicator LED.
static constexpr gpio_num_t LED_BOOTLOADER_PIN_NUM = GPIO_NUM_2;

/// GPIO Pin used for Factory Reset Button.
static constexpr gpio_num_t BUTTON_FACTORY_RESET_PIN_NUM = GPIO_NUM_3;

/// GPIO Pin used for Bootloader Button.
static constexpr gpio_num_t BUTTON_BOOTLOADER_PIN_NUM = GPIO_NUM_8;

/// Node Activity indicator LED. Active (ON) Low.
GPIO_PIN(LED_ACTIVITY, GpioOutputSafeHighInvert, LED_ACTIVITY_PIN_NUM);

/// Bootloader Write indicator LED. Active (ON) Low.
GPIO_PIN(LED_BOOTLOADER_WRITE, GpioOutputSafeHighInvert, LED_BOOTLOADER_WRITE_PIN_NUM);

/// Bootloader Active indicator LED. Active (ON) Low.
GPIO_PIN(LED_BOOTLOADER, GpioOutputSafeHighInvert, LED_BOOTLOADER_PIN_NUM);

/// Factory Reset Pin, Pull-Up enabled by default.
GPIO_PIN(FACTORY_RESET_BUTTON, GpioInputPU, BUTTON_FACTORY_RESET_PIN_NUM);

/// Bootloader Request Button Pin. Pull-Up enabled by default.
GPIO_PIN(BOOTLOADER_BUTTON, GpioInputPU, BUTTON_BOOTLOADER_PIN_NUM);

#endif // HARDWARE_HXX_