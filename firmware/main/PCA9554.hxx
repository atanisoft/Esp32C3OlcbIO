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
 * \file PCA9554.hxx
 * 
 * ESP32 based hardware driver for the PCA9554 IO extender chip.
 *
 * @author Mike Dunston
 * @date 24 May 2021
 */

#ifndef PCA9554_HXX_
#define PCA9554_HXX_

#include <driver/i2c.h>
#include <executor/Executor.hxx>
#include <mutex>
#include <openlcb/RefreshLoop.hxx>
#include <os/Gpio.hxx>
#include <utils/Atomic.hxx>
#include <utils/logging.h>

/// PCA9554 IO extender chip implementation.
///
/// This will poll the PCA9554 IC at roughly a 50Hz frequency.
///
/// The interrupt and reset lines are not used.
class PCA9554 : public openlcb::Polling
{
public:
    /// Constructor.
    ///
    /// @param address is the I2C address for the PCA9554.
    /// @param port is the I2C port to use.
    PCA9554(uint8_t address, i2c_port_t port = I2C_NUM_0)
        : i2cPort_(port)
        , i2cAddress_(address << 1)
    {
    }

    bool verify_present()
    {
        LOG(INFO, "[PCA9554 0x%02x] Verifying presence...", i2cAddress_ >> 1);
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, i2cAddress_ | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t res = i2c_master_cmd_begin(i2cPort_, cmd, I2C_TIMEOUT);
        i2c_cmd_link_delete(cmd);
        if (res != ESP_OK)
        {
            LOG_ERROR("[PCA9554 0x%02x] Device did not respond: %d (%s)"
                    , i2cAddress_ >> 1, res, esp_err_to_name(res));
        }
        else
        {
            // read initial input state
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, i2cAddress_ | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, INPUTS, true);
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, i2cAddress_ | I2C_MASTER_READ, true);
            i2c_master_read_byte(cmd, &gpio_, I2C_MASTER_NACK);
            i2c_master_read_byte(cmd, &lat_, I2C_MASTER_NACK);
            i2c_master_read_byte(cmd, &polarity_, I2C_MASTER_NACK);
            i2c_master_read_byte(cmd, &dir_, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            res = i2c_master_cmd_begin(i2cPort_, cmd, I2C_TIMEOUT);
            i2c_cmd_link_delete(cmd);
        }
        enabled_ = (res == ESP_OK);
        return enabled_;
    }

    void poll_33hz(openlcb::WriteHelper *helper, Notifiable *done) override
    {
        AutoNotify an(done);
        if (enabled_)
        {
            update();
        }
    }

private:
    DISALLOW_COPY_AND_ASSIGN(PCA9554);

    friend class PCA9554Gpio;

    enum Registers
    {
        INPUTS   = 0x0,
        OUTPUTS  = 0x1,
        POLARITY = 0x2,
        CONTROL  = 0x3
    };

    enum
    {
        /// Flag to indicate that the direction bits are dirty.
        DIRTY_DIR = 1,

        /// Flag to indicate that the latch state bits are dirty.
        DIRTY_LATCH = 2,

        /// Flag to indicate that the polarity state bits are dirty.
        DIRTY_POLARITY = 4,
    };

    /// Updates the state of the PCA9554 pins.
    void update()
    {
        bool push_dir = false;
        bool push_lat = false;
        bool push_polarity = false;
        uint8_t dir;
        uint8_t latch;
        uint8_t polarity;
        {
            const std::lock_guard<std::mutex> lock(mux_);
            dir = dir_;
            latch = lat_;
            polarity = polarity_;
            if (dirty_ & DIRTY_DIR)
            {
                dirty_ &= ~DIRTY_DIR;
                push_dir = true;
            }
            if (dirty_ & DIRTY_LATCH)
            {
                dirty_ &= ~DIRTY_LATCH;
                push_lat = true;
            }
            if (dirty_ & DIRTY_POLARITY)
            {
                dirty_ &= ~DIRTY_POLARITY;
                push_polarity = true;
            }
        }

        if (push_lat)
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, i2cAddress_ | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, OUTPUTS, true);
            i2c_master_write_byte(cmd, latch, true);
            i2c_master_stop(cmd);
            esp_err_t res = i2c_master_cmd_begin(i2cPort_, cmd, I2C_TIMEOUT);
            i2c_cmd_link_delete(cmd);
            if (res != ESP_OK)
            {
                LOG_ERROR("[PCA9554 0x%02x] I2C Failure %d (%s)", res
                        , i2cAddress_ >> 1, esp_err_to_name(res));
            }
        }
        if (push_polarity)
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, i2cAddress_ | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, POLARITY, true);
            i2c_master_write_byte(cmd, polarity, true);
            i2c_master_stop(cmd);
            esp_err_t res = i2c_master_cmd_begin(i2cPort_, cmd, I2C_TIMEOUT);
            i2c_cmd_link_delete(cmd);
            if (res != ESP_OK)
            {
                LOG_ERROR("[PCA9554 0x%02x] I2C Failure %d (%s)", res
                        , i2cAddress_ >> 1, esp_err_to_name(res));
            }
        }
        if (push_dir)
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, i2cAddress_ | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, CONTROL, true);
            i2c_master_write_byte(cmd, dir, true);
            i2c_master_stop(cmd);
            esp_err_t res = i2c_master_cmd_begin(i2cPort_, cmd, I2C_TIMEOUT);
            i2c_cmd_link_delete(cmd);
            if (res != ESP_OK)
            {
                LOG_ERROR("[PCA9554 0x%02x] I2C Failure %d (%s)", res
                        , i2cAddress_ >> 1, esp_err_to_name(res));
            }
        }

        {
            const std::lock_guard<std::mutex> lock(mux_);
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, i2cAddress_ | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, INPUTS, true);
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, i2cAddress_ | I2C_MASTER_READ, true);
            i2c_master_read_byte(cmd, &gpio_, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            esp_err_t res = i2c_master_cmd_begin(i2cPort_, cmd, I2C_TIMEOUT);
            i2c_cmd_link_delete(cmd);
            if (res != ESP_OK)
            {
                LOG_ERROR("[PCA9554 0x%02x] I2C Failure %d (%s)", res
                        , i2cAddress_ >> 1, esp_err_to_name(res));
            }
        }
    }

    static constexpr TickType_t I2C_TIMEOUT = pdMS_TO_TICKS(1);

    /// Internal flag to indicate that the I2C device has been found on the bus
    /// and is usable. When false the update loop will be skipped.
    bool enabled_{false};

    /// I2C port to be used for communicating with the PCA9554 device.
    i2c_port_t i2cPort_;

    /// Address of this particular device on the I2C port.
    uint8_t i2cAddress_;

    /// Shadow of the direction register.
    uint8_t dir_{0xFF};

    /// Shadow of the latch register.
    uint8_t lat_{0x00};

    /// Shadow of the input register.
    uint8_t gpio_{0x00};

    /// Shadow of the polarity register.
    uint8_t polarity_{0x00};

    /// Bit mask of registers that need updating.
    uint8_t dirty_{0};

    /// Mutex protecting the internal shadow registers.
    std::mutex mux_;
};

class PCA9554Gpio : public Gpio
{
public:
    constexpr PCA9554Gpio(PCA9554 *const parent, unsigned pin)
        : parent_(parent)
        , pinBit_(1 << pin)
    {
    }

    void set() const override
    {
        write(Value::VHIGH);
    }

    void clr() const override
    {
        write(Value::VLOW);
    }

    void write(Value new_state) const override
    {
        const std::lock_guard<std::mutex> lock(parent_->mux_);
        bool old_state = !!(parent_->lat_ & pinBit_);
        if (new_state != old_state)
        {
            if (new_state)
            {
                parent_->lat_ |= pinBit_;
            }
            else
            {
                parent_->lat_ &= ~pinBit_;
            }
            parent_->dirty_ |= parent_->DIRTY_LATCH;
        }
    }

    Value read() const override
    {
        if (parent_->dir_ & pinBit_)
        {
            // input. Use gpio_.
            return (parent_->gpio_ & pinBit_) ? Value::VHIGH : Value::VLOW;
        }
        else
        {
            // output. Use lat_.
            return (parent_->lat_ & pinBit_) ? VHIGH : VLOW;
        }
    }

    void set_direction(Direction dir) const override
    {
        const std::lock_guard<std::mutex> lock(parent_->mux_);
        uint8_t desired = (dir == Direction::DOUTPUT) ? 0 : pinBit_;
        if (desired != (parent_->dir_ & pinBit_))
        {
            parent_->dir_ =
                (parent_->dir_ & (~pinBit_)) | desired;
            parent_->dirty_ |= parent_->DIRTY_DIR;
        }
    }

    Direction direction() const override
    {
        if (parent_->dir_ & pinBit_)
        {
            return Direction::DINPUT;
        }
        else
        {
            return Direction::DOUTPUT;
        }
    }

    void set_polarity(uint8_t polarity) const
    {
        const std::lock_guard<std::mutex> lock(parent_->mux_);
        if (polarity)
        {
            parent_->polarity_ |= pinBit_;
        }
        else
        {
            parent_->polarity_ &= pinBit_;
        }
        parent_->dirty_ |= parent_->DIRTY_POLARITY;
    }

private:
    DISALLOW_COPY_AND_ASSIGN(PCA9554Gpio);

    /// @ref PCA9554 device this @ref Gpio instance belongs to.
    PCA9554 *const parent_;

    /// one bit that denotes the pin.
    const uint8_t pinBit_;
};

#endif // PCA9554_HXX_