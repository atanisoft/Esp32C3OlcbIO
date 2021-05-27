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
 * \file NodeIdMemoryConfigSpace.hxx
 *
 * Configured GPIO expander event handler.
 *
 * @author Mike Dunston
 * @date 23 May 2021
 */

#ifndef NODEID_MEMORY_CONFIG_SPACE_HXX_
#define NODEID_MEMORY_CONFIG_SPACE_HXX_

#include <openlcb/SimpleStack.hxx>
#include <openlcb/VirtualMemorySpace.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <utils/Singleton.hxx>

#include "nvs_config.hxx"

template<const unsigned num, const char separator>
void inject_seperator(std::string & input);
uint64_t string_to_uint64(std::string value);

namespace esp32c3io
{

CDI_GROUP(NodeIdConfig);
CDI_GROUP_ENTRY(node_id, openlcb::StringConfigEntry<32>);
CDI_GROUP_END();

/// Node configuration holder
NodeIdConfig node_id_config(0);

/// Virtual memory space that allows reconfiguration of the persistent node
/// identifier.
class NodeIdMemoryConfigSpace
    : public openlcb::VirtualMemorySpace
    , public Singleton<NodeIdMemoryConfigSpace>
{
public:
    /// Constructor.
    ///
    /// @param stack is the @ref SimpleCanStack that this memory space should
    /// be registered with.
    /// @param node_id is the current node identifier.
    NodeIdMemoryConfigSpace(openlcb::SimpleCanStack *stack, uint64_t node_id)
      : id_(uint64_to_string_hex(node_id, 12)), nodeid_(node_id)
    {
        std::replace(id_.begin(), id_.end(), ' ', '0');
        inject_seperator<2, '.'>(id_);
        register_string(node_id_config.node_id(),
            [&](unsigned repeat, string *contents, BarrierNotifiable *done)
            {
                LOG(INFO, "[NodeIdMemCfg-READ] %s", id_.c_str());
                *contents = id_;
                done->notify();
            },
            [&](unsigned repeat, string contents, BarrierNotifiable *done)
            {
                LOG(INFO, "[NodeIdMemCfg-WRITE] %s", contents.c_str());
                uint64_t new_node_id = string_to_uint64(contents);
                updated_ = set_node_id(new_node_id);
                nodeid_ = new_node_id;
                id_ = std::move(contents);
                done->notify();
            }
        );
        LOG(INFO, "[NodeIdMemCfg:%02x] NodeID: %s", SPACE, id_.c_str());
        stack->memory_config_handler()->registry()->insert(
            stack->node(), SPACE, this);
    }

    /// Returns the currently configured node identifier.
    uint64_t node_id()
    {
        return nodeid_;
    }

    /// @return true if the node identifier has been changed via this virtual
    /// memory space, false otherwise.
    bool updated()
    {
        return updated_;
    }
private:
    /// temporary holder for the node id in a hex string format.
    /// NOTE: the value will be a dot expanded hex format,
    /// ie: 05.02.01.03.10.00.
    std::string id_;

    /// temporary holder for the currently assigned node id.
    uint64_t nodeid_{0};

    /// Flag indicating that the node-id has been changed via the memory space.
    bool updated_{false};

    /// Memory space number where this space is registered.
    const uint8_t SPACE = 0xAB;
};

} // namespace esp32c3io

#endif // NODEID_MEMORY_CONFIG_SPACE_HXX_
