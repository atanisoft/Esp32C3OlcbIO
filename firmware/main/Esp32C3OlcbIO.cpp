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
 * \file Esp32C3OlcbIO.cpp
 *
 * Program entry point for the Esp32C3OlcbIO Board.
 *
 * @author Mike Dunston
 * @date 23 May 2021
 */
#include "sdkconfig.h"
#include "cdi.hxx"
#include "ConfiguredPinHandler.hxx"
#include "EventBroadcastHelper.hxx"
#include "FactoryResetHelper.hxx"
#include "fs.hxx"
#include "hardware.hxx"
#include "NodeIdMemoryConfigSpace.hxx"
#include "NodeRebootHelper.hxx"
#include "nvs_config.hxx"
#include "PCA9554.hxx"
#include <CDIXMLGenerator.hxx>
#include <freertos_drivers/esp32/Esp32HardwareTwai.hxx>
#include <freertos_drivers/esp32/Esp32BootloaderHal.hxx>
#include <freertos_drivers/esp32/Esp32SocInfo.hxx>
#include <openlcb/SimpleStack.hxx>
#include <utils/constants.hxx>
#include <utils/format_utils.hxx>
#include <utils/GpioInitializer.hxx>
#include <utils/logging.h>
#include <utils/Singleton.hxx>
#include <utils/Uninitialized.hxx>

/// Enables the printing of all packets in the CanHub. This is not enabled by
/// default due to performance implications and should only be used for
/// debugging.
static constexpr bool ENABLE_PACKET_PRINTER = false;

/// Number of seconds to hold the Factory Reset button to force clear all
/// stored configuration data.
static constexpr uint8_t FACTORY_RESET_HOLD_TIME = 10;

/// Number of seconds to hold the Factory Reset button to force regeneration of
/// all Event IDs. NOTE: This will *NOT* clear WiFi configuration data.
static constexpr uint8_t FACTORY_RESET_EVENTS_HOLD_TIME = 5;

/// Default address for the PCA9554 PWM IC (all address pins to GND).
static constexpr uint8_t PCA9554_ADDR = 0x38;

/// GPIO Pin initializer.
typedef GpioInitializer<LED_ACTIVITY_Pin, LED_BOOTLOADER_WRITE_Pin,
                        LED_BOOTLOADER_Pin, FACTORY_RESET_BUTTON_Pin,
                        BOOTLOADER_BUTTON_Pin> GpioInit;

PCA9554 pca9554(PCA9554_ADDR);
PCA9554Gpio pin_0(&pca9554, 0);
PCA9554Gpio pin_1(&pca9554, 1);
PCA9554Gpio pin_2(&pca9554, 2);
PCA9554Gpio pin_3(&pca9554, 3);
PCA9554Gpio pin_4(&pca9554, 4);
PCA9554Gpio pin_5(&pca9554, 5);
PCA9554Gpio pin_6(&pca9554, 6);
PCA9554Gpio pin_7(&pca9554, 7);
constexpr const PCA9554Gpio *const pca9554Gpio[] =
{
    &pin_0, &pin_1, &pin_2, &pin_3,
    &pin_4, &pin_5, &pin_6, &pin_7
};

Esp32HardwareTwai twai(TWAI_RX_PIN_NUM, TWAI_TX_PIN_NUM);
esp32c3io::ConfigDef cfg(0);
static uint32_t RTC_NOINIT_ATTR bootloader_request;

// Helper which converts a string to a uint64 value.
uint64_t string_to_uint64(std::string value)
{
  // remove period characters if present
  value.erase(std::remove(value.begin(), value.end(), '.'), value.end());
  // convert the string to a uint64_t value
  return std::stoull(value, nullptr, 16);
}

template<const unsigned num, const char separator>
void inject_seperator(std::string & input)
{
    for (auto it = input.begin(); (num + 1) <= std::distance(it, input.end());
        ++it)
    {
        std::advance(it, num);
        it = input.insert(it, separator);
    }
}

namespace openlcb
{
    /// Name of CDI.xml to generate dynamically.
    const char CDI_FILENAME[] = "/fs/cdi.xml";

    /// This will stop openlcb from exporting the CDI memory space upon start.
    extern const char CDI_DATA[] = "";

    /// Path to where OpenMRN should persist general configuration data.
    extern const char *const CONFIG_FILENAME = "/fs/openlcb_config";

    /// The size of the memory space to export over the above device.
    extern const size_t CONFIG_FILE_SIZE =
        cfg.seg().size() + cfg.seg().offset();

    /// Default to store the dynamic SNIP data is stored in the same persistant
    /// data file as general configuration data.
    extern const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;

    /// Defines the identification information for the node. The arguments are:
    ///
    /// - 4 (version info, always 4 by the standard
    /// - Manufacturer name
    /// - Model name
    /// - Hardware version
    /// - Software version
    ///
    /// This data will be used for all purposes of the identification:
    ///
    /// - the generated cdi.xml will include this data
    /// - the Simple Node Ident Info Protocol will return this data
    /// - the ACDI memory space will contain this data.
    const SimpleNodeStaticValues SNIP_STATIC_DATA =
    {
        4,
        SNIP_PROJECT_PAGE,
        SNIP_PROJECT_NAME,
        SNIP_HW_VERSION,
        SNIP_SW_VERSION
    };

    /// Modify this value every time the EEPROM needs to be cleared on the node
    /// after an update.
    static constexpr uint16_t CANONICAL_VERSION = CDI_VERSION;
} // namespace openlcb

extern "C"
{

void resetblink(uint32_t pattern)
{
    LED_BOOTLOADER_WRITE_Pin::instance()->write(pattern);
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}

void __attribute__((noreturn)) diewith(uint32_t pattern)
{
    uint32_t p = 0;
    while(true)
    {
        LED_BOOTLOADER_WRITE_Pin::instance()->write(p & 1);
        p >>= 1;
        if (!p) p = pattern;
        ets_delay_us(125000);
    }
}

void *node_reboot(void *arg)
{
    Singleton<esp32c3io::NodeRebootHelper>::instance()->reboot();
    return nullptr;
}

void reboot()
{
    os_thread_create(nullptr, nullptr, uxTaskPriorityGet(NULL) + 1, 2048
                   , node_reboot, nullptr);
}

/// Initializes the node specific bootloader hardware (LEDs)
void bootloader_hw_set_to_safe(void)
{
    LOG(VERBOSE, "[Bootloader] bootloader_hw_set_to_safe");
    LED_BOOTLOADER_WRITE_Pin::hw_init();
    LED_BOOTLOADER_Pin::hw_init();
}

/// Requests that the node enters the OpenLCB Bootloader mode.
void enter_bootloader()
{
    // set global flag that we need to enter the bootloader
    bootloader_request = 1;
    LOG(INFO, "[Bootloader] Rebooting into bootloader");
    // reboot so we can enter the bootloader
    esp_restart();
}

/// Verifies that the bootloader has been requested.
///
/// @return true if bootloader_request is set to one, otherwise false.
bool request_bootloader(void)
{
  LOG(VERBOSE, "[Bootloader] request_bootloader");
  return bootloader_request;
}

/// Updates the state of a status LED.
///
/// @param led is the LED to update.
/// @param value is the new state of the LED.
///
/// NOTE: Currently the following mapping is being used for the LEDs:
/// LED_ACTIVE -> Bootloader LED
/// LED_WRITING -> Bootloader Write LED
/// LED_REQUEST -> Used only as a hook for printing bootloader startup.
void bootloader_led(enum BootloaderLed led, bool value)
{
    LOG(VERBOSE, "[Bootloader] bootloader_led(%d, %d)", led, value);
    if (led == LED_ACTIVE)
    {
        LED_BOOTLOADER_Pin::instance()->write(value);
    }
    else if (led == LED_WRITING)
    {
        LED_BOOTLOADER_WRITE_Pin::instance()->write(value);
    }
    else if (led == LED_REQUEST)
    {
        LOG(INFO, "[Bootloader] Preparing to receive firmware");
        LOG(INFO, "[Bootloader] Current partition: %s", current->label);
        LOG(INFO, "[Bootloader] Target partition: %s", target->label);
    }
}

void i2c_setup(int sda, int scl, bool scan = true)
{
    i2c_config_t i2c_config;
    bzero(&i2c_config, sizeof(i2c_config_t));
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = sda;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_io_num = scl;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = 100000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    if (scan)
    {
        // Scan the I2C bus and dump the output of devices that respond
        std::string scanresults =
            "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n"
            "00:         ";
        scanresults.reserve(256);
        for (uint8_t addr=3; addr < 0x78; addr++)
        {
            if (addr % 16 == 0)
            {
                scanresults += "\n" + int64_to_string_hex(addr) + ":";
            }
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t ret =
                i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK)
            {
                scanresults += StringPrintf(" %02x", addr);
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                scanresults += " ??";
            }
            else
            {
                scanresults += " --";
            }
        }
        LOG(INFO, "[I2C] Scan results:\n%s", scanresults.c_str());
    }
}

void app_main()
{
    uint8_t reset_reason = Esp32SocInfo::print_soc_info();

    // If this is the first power up of the node we need to reset the flag
    // since it will not be initialized automatically.
    if (reset_reason == POWERON_RESET)
    {
        bootloader_request = 0;
    }

    LOG(INFO, "[SNIP] version:%d, manufacturer:%s, model:%s, hw-v:%s, sw-v:%s"
      , openlcb::SNIP_STATIC_DATA.version
      , openlcb::SNIP_STATIC_DATA.manufacturer_name
      , openlcb::SNIP_STATIC_DATA.model_name
      , openlcb::SNIP_STATIC_DATA.hardware_version
      , openlcb::SNIP_STATIC_DATA.software_version);
    bool reset_events = false;
    GpioInit::hw_init();

    nvs_init();

    // load non-CDI based config from NVS.
    bool cleanup_config_tree = false;
    node_config_t config;
    if (load_config(&config) != ESP_OK)
    {
        default_config(&config);
        cleanup_config_tree = true;
    }
    else if (config.force_reset)
    {
        LOG(WARNING,
            "[NVS] Force reset flag is set, configuration data will be cleared");
        cleanup_config_tree = true;
    }

    // Check for factory reset button being held to GND and the BOOTLOADER
    // button not being held to GND. If this is detected the factory reset
    // process will be started.
    if (FACTORY_RESET_BUTTON_Pin::instance()->is_clr() &&
        BOOTLOADER_BUTTON_Pin::instance()->is_set())
    {
        // Set the LEDs in an on/off/on pattern for the blink
        LED_ACTIVITY_Pin::instance()->set();
        LED_BOOTLOADER_WRITE_Pin::instance()->clr();
        LED_BOOTLOADER_Pin::instance()->set();

        // Count down from the overall factory reset time.
        uint8_t hold_time = FACTORY_RESET_HOLD_TIME;
        for (; hold_time > 0 && FACTORY_RESET_BUTTON_Pin::instance()->is_clr();
             hold_time--)
        {
            if (hold_time > FACTORY_RESET_EVENTS_HOLD_TIME)
            {
                LOG(WARNING
                  , "Event ID reset in %d seconds, factory reset in %d seconds."
                  , hold_time - FACTORY_RESET_EVENTS_HOLD_TIME, hold_time);
                LED_ACTIVITY_Pin::toggle();
                LED_BOOTLOADER_Pin::toggle();
            }
            else
            {
                LED_ACTIVITY_Pin::instance()->clr();
                LED_BOOTLOADER_Pin::instance()->clr();
                LOG(WARNING, "Factory reset in %d seconds.", hold_time);
            }
            usleep(SEC_TO_USEC(1));
            LED_BOOTLOADER_WRITE_Pin::toggle();
        }
        if (FACTORY_RESET_BUTTON_Pin::instance()->is_clr() && hold_time == 0)
        {
            // if the button is still being held and the hold time expired
            // start a full factory reset.
            LOG(WARNING, "Factory reset triggered!");
            cleanup_config_tree = true;
        }
        else if (hold_time <= FACTORY_RESET_EVENTS_HOLD_TIME)
        {
            // if the button is not being held and the hold time is less than
            // the event id reset count down trigger a reset of events.
            LOG(WARNING, "Reset of events triggered!");
            reset_events = true;
        }
        else
        {
            // The button was released prior to the event id reset limit, do
            // nothing.
            LOG(WARNING, "Factory reset aborted!");
        }
    }
    else if (BOOTLOADER_BUTTON_Pin::instance()->is_clr())
    {
        // If both the factory reset and user button are held to GND it is a
        // request to enter the bootloader mode.
        bootloader_request = 1;

        // give a visual indicator that the bootloader request has been ACK'd
        // turn on both Bootloader and Activity LEDs, wait ~1sec, turn off
        // Bootloader LED, wait ~1sec, turn off Activity LED.
        LED_ACTIVITY_Pin::instance()->set();
        LED_BOOTLOADER_Pin::instance()->set();
        vTaskDelay(pdMS_TO_TICKS(1000));
        LED_BOOTLOADER_Pin::instance()->clr();
        vTaskDelay(pdMS_TO_TICKS(1000));
        LED_ACTIVITY_Pin::instance()->clr();
    }

    // reset the LEDs back to default settings
    LED_ACTIVITY_Pin::instance()->clr();
    LED_BOOTLOADER_Pin::instance()->clr();
    LED_BOOTLOADER_WRITE_Pin::instance()->clr();

    // if we have a request to enter the bootloader we need to process it
    // before we startup the OpenMRN stack.
    if (bootloader_request)
    {
        esp32_bootloader_run(config.node_id, TWAI_RX_PIN_NUM, TWAI_TX_PIN_NUM,
            false);
        bootloader_request = 0;
        esp_restart();
    }
    else
    {
        dump_config(&config);
        mount_fs(cleanup_config_tree);

        i2c_setup(SDA_PIN_NUM, SCL_PIN_NUM, true);

        openlcb::SimpleCanStack stack(config.node_id);
        stack.set_tx_activity_led(LED_ACTIVITY_Pin::instance());

        if (ENABLE_PACKET_PRINTER)
        {
            stack.print_all_packets();
        }

        // Create / update CDI, if the CDI is out of date a factory reset will be
        // forced.
        bool reset_cdi = CDIXMLGenerator::create_config_descriptor_xml(
            cfg, openlcb::CDI_FILENAME, &stack);
        if (reset_cdi)
        {
            LOG(WARNING, "[CDI] Forcing factory reset due to CDI update");
            unlink(openlcb::CONFIG_FILENAME);
        }

        esp32c3io::FactoryResetHelper factory_reset_helper(cfg.userinfo());
        esp32c3io::NodeIdMemoryConfigSpace virtual_space(&stack, config.node_id);
        esp32c3io::EventBroadcastHelper event_helper(&stack);
        esp32c3io::ConfiguredPinHandler pin_handler(
            stack.node(), pca9554Gpio, ARRAYSIZE(pca9554Gpio),
            cfg.seg().gpio());
        openlcb::RefreshLoop refresh_loop(
            stack.node(), { &pca9554, &pin_handler });
        pca9554.verify_present();

        // Create config file and initiate factory reset if it doesn't exist
        // or is otherwise corrupted.
        int config_fd =
            stack.create_config_file_if_needed(cfg.seg().internal_config(),
                openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);
        esp32c3io::NodeRebootHelper reboot_helper(&stack, config_fd);

        if (reset_events)
        {
            LOG(WARNING, "[CDI] Resetting event IDs");
            stack.factory_reset_all_events(
                cfg.seg().internal_config(), config.node_id, config_fd);
            fsync(config_fd);
        }

        // initialize the TWAI driver.
        twai.hw_init();

        // add TWAI driver with select() usage.
        stack.add_can_port_select("/dev/twai/twai0");

        // if a brownout was detected send an event as part of node startup.
        if (reset_reason == RTCWDT_BROWN_OUT_RESET)
        {
            event_helper.send_event(openlcb::Defs::NODE_POWER_BROWNOUT_EVENT);
        }

        // Start the OpenMRN stack
        stack.loop_executor();
    }
}

} // extern "C"