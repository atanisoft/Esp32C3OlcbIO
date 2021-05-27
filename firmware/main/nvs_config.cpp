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
 * \file nvs_config.cpp
 *
 * NVS based configuration management for the Esp32C3OlcbIO Board.
 *
 * @author Mike Dunston
 * @date 23 May 2021
 */

#include "nvs_config.hxx"
#include "sdkconfig.h"
#include "hardware.hxx"

#include <nvs.h>
#include <nvs_flash.h>
#include <string>
#include <algorithm>

// TODO: adjust format_utils.hxx not to require this line here.
using std::string;

#include <utils/blinker.h>
#include <utils/format_utils.hxx>
#include <utils/logging.h>

// long blink, long blink
#define NVS_BLINK_VFS_FAILED  0b1100

/// NVS Persistence namespace.
static constexpr char NVS_NAMESPACE[] = "nodecfg";

/// NVS Persistence key.
static constexpr char NVS_CFG_KEY[] = "cfg";

/// Default Node ID that will be assigned upon Factory Reset.
static constexpr uint64_t DEFAULT_NODE_ID = 0x050201031000;

/// Loads the node configuration from NVS.
///
/// @param config is the holder for the loaded node configuration.
/// @return ESP_OK if the configuration has been successfully loaded,
/// other values indicate a failure.
esp_err_t load_config(node_config_t *config)
{
    LOG(INFO, "[NVS] Loading configuration");
    // load non-CDI based config from NVS
    nvs_handle_t nvs;
    size_t size = sizeof(node_config_t);
    esp_err_t res =
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs));
    if (res != ESP_OK)
    {
        LOG_ERROR("[NVS] Configuration load failed: %s (%d)"
                , esp_err_to_name(res), res);
        return res;
    }
    res = nvs_get_blob(nvs, NVS_CFG_KEY, config, &size);
    nvs_close(nvs);

    // if the size read in is not as expected reset the result code to failure.
    if (size != sizeof(node_config_t))
    {
        LOG_ERROR("[NVS] Configuration load failed (loaded size incorrect: "
                  "%zu vs %zu)", size, sizeof(node_config_t));
        res = ESP_FAIL;
    }
    return res;
}

/// Persists the node configuration into NVS.
///
/// @param config is the configuration data to persist.
/// @return ESP_OK if the configuration data has been persisted successfully,
/// other values indicate a failure.
esp_err_t save_config(node_config_t *config)
{
    nvs_handle_t nvs;
    esp_err_t res =
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs));
    if (res != ESP_OK)
    {
        LOG_ERROR("[NVS] Configuration save failed: %s (%d)"
                , esp_err_to_name(res), res);
        return res;
    }
    res =
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            nvs_set_blob(nvs, NVS_CFG_KEY, config, sizeof(node_config_t)));
    if (res != ESP_OK)
    {
        LOG_ERROR("[NVS] Configuration save failed: %s (%d)"
                , esp_err_to_name(res), res);
        return res;
    }
    res = ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(nvs));
    nvs_close(nvs);
    if (res != ESP_OK)
    {
        LOG_ERROR("[NVS] Commit failed: %s (%d)", esp_err_to_name(res), res);
    }
    else
    {
        LOG(INFO, "[NVS] Configuration saved");
    }
    return res;
}

/// Initializes the configuration data structure with default values.
///
/// @param config is the configuration data structure to initialize.
/// @return ESP_OK if the configuration has been persisted, other values
/// indicate a failure.
esp_err_t default_config(node_config_t *config)
{
    LOG(INFO, "[NVS] Initializing default configuration");
    bzero(config, sizeof(node_config_t));
    config->node_id = DEFAULT_NODE_ID;
    return save_config(config);
}

/// Initializes the NVS system.
///
/// If the NVS has not previously been initialized the storage partition will
/// be formatted prior to initialization. If there is no NVS partition or there
/// is a failure during initialization the bootloader LED will emit a series of
/// long blinks and halt the startup of the node.
void nvs_init()
{
    // Initialize NVS before we do any other initialization as it may be
    // internally used by various components even if we disable it's usage in
    // the WiFi connection stack.
    LOG(INFO, "[NVS] Initializing NVS");
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_init()) == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        const esp_partition_t *part =
            esp_partition_find_first(ESP_PARTITION_TYPE_DATA
                                   , ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
        if (part != NULL)
        {
            LOG(INFO, "[NVS] Erasing partition %s...", part->label);
            ESP_ERROR_CHECK(esp_partition_erase_range(part, 0, part->size));
            ESP_ERROR_CHECK(nvs_flash_init());
        }
        else
        {
            LOG_ERROR("[NVS] Unable to locate NVS partition!");
            diewith(NVS_BLINK_VFS_FAILED);
        }
    }
    else
    {
        LOG_ERROR("[NVS] Unable to locate NVS partition!");
        diewith(NVS_BLINK_VFS_FAILED);
    }
}

template<const unsigned num, const char separator>
void inject_seperator(std::string & input);

/// Displays the persistent NVS configuration data.
///
/// @param config is the persistent configuration to display.
void dump_config(node_config_t *config)
{
    string node_id = uint64_to_string_hex(config->node_id, 12);
    std::replace(node_id.begin(), node_id.end(), ' ', '0');
        inject_seperator<2, '.'>(node_id);
    LOG(INFO, "[NVS] Node ID: %s", node_id.c_str());
}

/// Forces a factory reset of the node on the next restart.
///
/// @return true if the node will factory reset on the next startup, false 
/// otherwise.
bool force_factory_reset()
{
    node_config_t config;
    load_config(&config);
    config.force_reset = true;

    return save_config(&config) == ESP_OK;
}

/// Reconfigures the persistent node identifier.
///
/// @param node_id is the new node identifier.
/// @return true if the new node identifier has been persisted, false
/// otherwise.
bool set_node_id(uint64_t node_id)
{
    node_config_t config;
    load_config(&config);
    string current_id = uint64_to_string_hex(config.node_id, 12);
    string new_id = uint64_to_string_hex(node_id, 12);
    std::replace(current_id.begin(), current_id.end(), ' ', '0');
    inject_seperator<2, '.'>(current_id);
    std::replace(new_id.begin(), new_id.end(), ' ', '0');
    inject_seperator<2, '.'>(new_id);
    LOG(INFO, "[NVS] Node ID: %s -> %s", current_id.c_str(), new_id.c_str());
    config.node_id = node_id;
    config.force_reset = true;

    return save_config(&config) == ESP_OK;
}