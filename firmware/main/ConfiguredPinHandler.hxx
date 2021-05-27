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
 * \file ConfiguredPinHandler.hxx
 *
 * Configured GPIO expander event handler.
 *
 * @author Mike Dunston
 * @date 23 May 2021
 */

#ifndef CONFIGURED_PIN_HANDLER_HXX_
#define CONFIGURED_PIN_HANDLER_HXX_

#include "cdi.hxx"
#include "PCA9554.hxx"
#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/RefreshLoop.hxx>
#include <os/Gpio.hxx>
#include <utils/ConfigUpdateListener.hxx>
#include <utils/ConfigUpdateService.hxx>
#include <utils/Debouncer.hxx>
#include <utils/format_utils.hxx>

namespace esp32c3io
{

class ConfiguredPinHandler :
    public ConfigUpdateListener, private openlcb::SimpleEventHandler,
    public openlcb::Polling, private Notifiable
{
public:
    typedef esp32c3io::PinConfig config_entry_type;
    typedef QuiesceDebouncer debouncer_type;

    /// Constructor.
    ///
    /// @param node is the OpenLCB node object from the stack.
    /// @param pins is the list of pins represented by the Gpio* object
    /// instances. Can be constant from FLASH space.
    /// @param size is the length of the list of pins array.
    /// @param config is the repeated group object from the configuration space
    /// that represents the locations of the events.
    template <unsigned N>
    __attribute__((noinline))
    ConfiguredPinHandler(openlcb::Node *node, const PCA9554Gpio *const *pins, unsigned size,
        const openlcb::RepeatedGroup<config_entry_type, N> &config)
        : node_(node)
        , pins_(pins)
        , size_(N)
        , offset_(config)
    {
        // Mismatched sizing of the GPIO array from the configuration array.
        HASSERT(size == N);
        ConfigUpdateService::instance()->register_update_listener(this);
        events_ = new openlcb::EventId[size * 2];
        std::allocator<debouncer_type> alloc;
        debouncers_ = alloc.allocate(size_);
        for (unsigned i = 0; i < size_; ++i)
        {
            alloc.construct(debouncers_ + i, 3);
        }
    }

    ~ConfiguredPinHandler()
    {
        do_unregister();
        ConfigUpdateService::instance()->unregister_update_listener(this);
        delete[] events_;
        std::allocator<debouncer_type> alloc;
        for (unsigned i = 0; i < size_; ++i)
        {
            alloc.destroy(debouncers_ + i);
        }
        alloc.deallocate(debouncers_, size_);
    }

    /// @return the instance to give to the RefreshLoop object.
    openlcb::Polling *polling()
    {
        return this;
    }

    /// Call from the refresh loop.
    void poll_33hz(openlcb::WriteHelper *helper, Notifiable *done) override
    {
        nextPinToPoll_ = 0;
        pollingHelper_ = helper;
        pollingDone_ = done;
        this->notify();
    }

    /// Asynchronous callback when the previous polling message has left via
    /// the bus. Used as a poor man's iterative state machine.
    void notify() override
    {
        for (; nextPinToPoll_ < size_; ++nextPinToPoll_)
        {
            auto i = nextPinToPoll_;
            if (pins_[i]->direction() == Gpio::Direction::DOUTPUT)
            {
                continue;
            }
            if (debouncers_[i].update_state(pins_[i]->is_set()))
            {
                // Pin flipped.
                ++nextPinToPoll_; // avoid infinite loop.
                auto event = events_[2 * i +
                    (debouncers_[i].current_state() ? 1 : 0)];
                pollingHelper_->WriteAsync(node_,
                    openlcb::Defs::MTI_EVENT_REPORT,
                    openlcb::WriteHelper::global(),
                    openlcb::eventid_to_buffer(event), this);
                return;
            }
        }
        pollingDone_->notify();
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        if (!initial_load)
        {
            // There is no way to figure out what the previously registered
            // eventid values were for the individual pins. Therefore we always
            // unregister everything and register them anew. It also causes us
            // to identify all. This is not a problem since apply_configuration
            // is coming from a user action.
            do_unregister();
        }
        openlcb::RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            const config_entry_type cfg_ref(grp_ref.entry(i));
            EventId cfg_event_on = cfg_ref.cfg().on().read(fd);
            EventId cfg_event_off = cfg_ref.cfg().off().read(fd);
            openlcb::EventRegistry::instance()->register_handler(
                openlcb::EventRegistryEntry(this, cfg_event_off, i * 2), 0);
            openlcb::EventRegistry::instance()->register_handler(
                openlcb::EventRegistryEntry(this, cfg_event_on, i * 2 + 1), 0);
            pins_[i]->set_polarity(cfg_ref.polarity().read(fd));
            uint8_t action = cfg_ref.mode().read(fd);
            if (action == (uint8_t)esp32c3io::PinConfig::ActionConfig::DOUTPUT)
            {
                pins_[i]->set_direction(Gpio::Direction::DOUTPUT);
                events_[i * 2] = 0;
                events_[i * 2 + 1] = 0;
            }
            else
            {
                uint8_t param = cfg_ref.input().debounce().read(fd);
                pins_[i]->set_direction(Gpio::Direction::DINPUT);
                debouncers_[i].reset_options(param);
                debouncers_[i].initialize(pins_[i]->read());
                events_[i * 2] = cfg_event_off;
                events_[i * 2 + 1] = cfg_event_on;
            }
        }
        return REINIT_NEEDED; // Causes events identify.
    }

    void factory_reset(int fd) OVERRIDE
    {
        openlcb::RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            grp_ref.entry(i).cfg().description().write(fd,
                StringPrintf("Pin %d", i + 1).c_str());
            CDI_FACTORY_RESET(grp_ref.entry(i).mode);
            CDI_FACTORY_RESET(grp_ref.entry(i).polarity);
            CDI_FACTORY_RESET(grp_ref.entry(i).output().pulse_count);
            CDI_FACTORY_RESET(grp_ref.entry(i).output().pulse_duration);
            CDI_FACTORY_RESET(grp_ref.entry(i).output().paired);
            CDI_FACTORY_RESET(grp_ref.entry(i).input().debounce);
        }
    }

    void handle_identify_global(const openlcb::EventRegistryEntry &registry_entry,
        openlcb::EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->dst_node && event->dst_node != node_)
        {
            return;
        }
        unsigned pin = registry_entry.user_arg >> 1;
        if (pins_[pin]->direction() == Gpio::Direction::DINPUT)
        {
            SendProducerIdentified(registry_entry, event, done);
        }
        else
        {
            SendConsumerIdentified(registry_entry, event, done);
        }
    }

    void handle_identify_consumer(const openlcb::EventRegistryEntry &registry_entry,
        openlcb::EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->event != registry_entry.event)
        {
            return;
        }
        unsigned pin = registry_entry.user_arg >> 1;
        if (pins_[pin]->direction() == Gpio::Direction::DOUTPUT)
        {
            SendConsumerIdentified(registry_entry, event, done);
        }
    }

    void handle_identify_producer(const openlcb::EventRegistryEntry &registry_entry,
        openlcb::EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->event != registry_entry.event)
        {
            return;
        }
        unsigned pin = registry_entry.user_arg >> 1;
        if (pins_[pin]->direction() == Gpio::Direction::DINPUT)
        {
            SendProducerIdentified(registry_entry, event, done);
        }
    }

    void handle_event_report(const openlcb::EventRegistryEntry &registry_entry,
        openlcb::EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->event != registry_entry.event)
        {
            return;
        }
        const Gpio *pin = pins_[registry_entry.user_arg >> 1];
        if (pin->direction() == Gpio::Direction::DOUTPUT)
        {
            const bool is_on = (registry_entry.user_arg & 1);
            pin->write(is_on);
        }
    }

private:
    /// Removes registration of this event handler from the global event
    /// registry.
    void do_unregister()
    {
        openlcb::EventRegistry::instance()->unregister_handler(this);
    }

    /// Sends out a ConsumerIdentified message for the given registration
    /// entry.
    void SendConsumerIdentified(
        const openlcb::EventRegistryEntry &registry_entry,
        openlcb::EventReport *event, BarrierNotifiable *done)
    {
        openlcb::Defs::MTI mti = openlcb::Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        unsigned b1 = pins_[registry_entry.user_arg >> 1]->is_set() ? 1 : 0;
        unsigned b2 = registry_entry.user_arg & 1; // on or off event?
        if (b1 ^ b2)
        {
            mti++; // INVALID
        }
        event->event_write_helper<3>()->WriteAsync(node_, mti,
            openlcb::WriteHelper::global(),
            openlcb::eventid_to_buffer(registry_entry.event),
            done->new_child());
    }

    /// Sends out a ProducerIdentified message for the given registration
    /// entry.
    void SendProducerIdentified(
        const openlcb::EventRegistryEntry &registry_entry,
        openlcb::EventReport *event, BarrierNotifiable *done)
    {
        openlcb::Defs::MTI mti = openlcb::Defs::MTI_PRODUCER_IDENTIFIED_VALID;
        unsigned b1 = pins_[registry_entry.user_arg >> 1]->is_set() ? 1 : 0;
        unsigned b2 = registry_entry.user_arg & 1; // on or off event?
        if (b1 ^ b2)
        {
            mti++; // INVALID
        }
        event->event_write_helper<4>()->WriteAsync(node_, mti,
            openlcb::WriteHelper::global(),
            openlcb::eventid_to_buffer(registry_entry.event),
            done->new_child());
    }

    // Variables used for asynchronous state during the polling loop.
    /// Which pin to next check when polling.
    unsigned nextPinToPoll_;
    /// Write helper to use for producing messages during the polling loop.
    openlcb::WriteHelper *pollingHelper_;
    /// Notifiable to call when the polling loop is done.
    Notifiable *pollingDone_;
    /// virtual node to export the consumer / producer on.
    openlcb::Node *node_;
    /// Array of all GPIO pins to use. Externally owned.
    const PCA9554Gpio *const *pins_;
    /// Number of GPIO pins to export.
    size_t size_;
    /// Offset in the configuration space for our configs.
    openlcb::ConfigReference offset_;
    /// Event IDs shadowing from the config file for producing them. We own
    /// this memory.
    openlcb::EventId *events_;
    /// One debouncer per pin, created for produced pins. We own this memory.
    debouncer_type *debouncers_;
};

} // namespace esp32c3io

#endif // CONFIGURED_PIN_HANDLER_HXX_