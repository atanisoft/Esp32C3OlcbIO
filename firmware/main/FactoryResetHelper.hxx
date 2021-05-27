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
 * \file FactoryResetHelper.hxx
 *
 * Factory Reset handler for Esp32C3OlcbIO Board.
 *
 * @author Mike Dunston
 * @date 23 May 2021
 */

#ifndef FACTORY_RESET_HELPER_HXX_
#define FACTORY_RESET_HELPER_HXX_

#include <utils/ConfigUpdateListener.hxx>
#include <utils/logging.h>

#include "NodeIdMemoryConfigSpace.hxx"

template<const unsigned num, const char separator>
void inject_seperator(std::string & input);
uint64_t string_to_uint64(std::string value);

namespace esp32c3io
{

class FactoryResetHelper : public DefaultConfigUpdateListener
{
public:
    FactoryResetHelper(const openlcb::UserInfoSegment &cfg) : cfg_(cfg)
    {
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) override
    {
        AutoNotify n(done);
        LOG(VERBOSE, "[CFG] apply_configuration(%d, %d)", fd, initial_load);

        // If this is not our initial load and the node id has changed we need
        // to force a reboot
        if (!initial_load &&
            Singleton<NodeIdMemoryConfigSpace>::instance()->updated())
        {
            LOG(WARNING,
                "[CFG] Node ID has been changed, node will reboot and factory "
                "reset to complete the update.");
            return ConfigUpdateListener::UpdateAction::REBOOT_NEEDED;
        }

        return ConfigUpdateListener::UpdateAction::UPDATED;
    }

    void factory_reset(int fd) override
    {
        LOG(INFO, "[CDI] Node configuration factory reset invoked.");
        // set the name of the node to the SNIP model name
        cfg_.name().write(fd, openlcb::SNIP_STATIC_DATA.model_name);
        std::string node_id =
            uint64_to_string_hex(Singleton<NodeIdMemoryConfigSpace>::instance()->node_id(), 12);
        std::replace(node_id.begin(), node_id.end(), ' ', '0');
        inject_seperator<2, '.'>(node_id);
        // set the node description to the node id in expanded hex format.
        cfg_.description().write(fd, node_id.c_str());
    }
private:
    const openlcb::UserInfoSegment cfg_;
};

} // namespace esp32c3io

#endif // FACTORY_RESET_HELPER_HXX_