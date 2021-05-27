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
 * \file cdi.hxx
 *
 * Config representation for the Esp32C3OlcbIO Board.
 *
 * @author Mike Dunston
 * @date 23 May 2021
 */

#ifndef CDI_HXX_
#define CDI_HXX_

#include "sdkconfig.h"

#include <freertos_drivers/esp32/Esp32WiFiConfiguration.hxx>
#include <openlcb/ConfigRepresentation.hxx>
#include <openlcb/MemoryConfig.hxx>

namespace esp32c3io
{

static const char PC_MODE_MAP[] =
    "<relation><property>0</property><value>Output</value></relation>"
    "<relation><property>1</property><value>Input</value></relation>";

static const char PC_POLARITY_MAP[] =
    "<relation><property>0</property><value>Normal</value></relation>"
    "<relation><property>1</property><value>Inverted</value></relation>";

static const char PC_ON_OFF_MAP[] =
    "<relation><property>0</property><value>Off</value></relation>"
    "<relation><property>1</property><value>On</value></relation>";

CDI_GROUP(IOPinConfig);
/// Allows the user to assign a name for this pin.
CDI_GROUP_ENTRY(description, openlcb::StringConfigEntry<20>, //
    Name("Description"), Description("User name of this pin."));
/// Specifies the event ID to use to turn the pin ON or raise when the pin is
/// ON.
CDI_GROUP_ENTRY(on, openlcb::EventConfigEntry, //
    Name("Event On"),
    Description("This event is used to turn the output ON or produced when "
                "the input turns ON."));
/// Specifies the event ID to use to turn the pin OFF or raise when the pin is
/// OFF.
CDI_GROUP_ENTRY(off, openlcb::EventConfigEntry, //
    Name("Event Off"),
    Description("This event is used to turn the output OFF or produced when "
                "the input turns OFF."));
CDI_GROUP_END();

CDI_GROUP(OutputConfiguration);
CDI_GROUP_ENTRY(pulse_count, openlcb::Uint8ConfigEntry,
    Default(0), Min(0), Max(255),
    Name("Pulse Count"),
    Description("This configures how many pulses to send on the ouput when "
                "it is ON.\nA value of 0 (zero) will disable sending a pulse "
                "and is treated as \"always on\".\nA value of 1-254 will send "
                "that number of pulses when the output is ON.\nA value of 255 "
                "will send continuous pulses until the output is turned OFF."));
/// Configures the pulse duration
CDI_GROUP_ENTRY(pulse_duration, openlcb::Uint8ConfigEntry,
    Default(0), Min(0), Max(255),
    Name("Pulse Duration"),
    Description("This configures how long each pulse will be, this is "
                "expressed in the number of 30 millisecond periods that each "
                "pulse should last."));
CDI_GROUP_ENTRY(paired, openlcb::Uint8ConfigEntry,
    Default(0), MapValues(PC_ON_OFF_MAP),
    Name("Paired Pin"),
    Description("Enabling this option will alter the behavior of the previous "
                "pin AND this pin. The behavior will be the previous pin "
                "\"ON\" event will apply to that pin and the \"OFF\" even will "
                "apply to this pin.\nNote: this setting will ALWAYS be ignored "
                "for pin 1."));
CDI_GROUP_END();

CDI_GROUP(InputConfiguration);
/// Configures the debounce parameter.
CDI_GROUP_ENTRY(debounce, openlcb::Uint8ConfigEntry,
    Default(3), Min(0), Max(255),
    Name("Debounce parameter"),
    Description("This configures how many successive reads of the input "
                "before raising the corresponding event. The frequency of "
                "reads is roughly every 30 milliseconds. A value of 2-3 will "
                "usually work well for a non-noisy environment. In some cases "
                "a value of 8-15 may be more desirable, this will increase "
                "the response time but may result in a more stable signal."));
CDI_GROUP_END();

CDI_GROUP(PinConfig);
enum class ActionConfig : uint8_t
{
    DOUTPUT = 0,
    DINPUT = 1
};

enum class PolarityConfig : uint8_t
{
    POLARITY_NORMAL = 0,
    POLARITY_INVERTED = 1
};

CDI_GROUP_ENTRY(mode, openlcb::Uint8ConfigEntry,    //
    Default((uint8_t)ActionConfig::DOUTPUT),        //
    MapValues(PC_MODE_MAP),                         //
    Name("Mode"),                                   //
    Description("This controls how the pin is configured."));

CDI_GROUP_ENTRY(polarity, openlcb::Uint8ConfigEntry,   //
    Default((uint8_t)PolarityConfig::POLARITY_NORMAL), //
    MapValues(PC_POLARITY_MAP),                        //
    Name("Polarity"),                                  //
    Description("This controls the polarity to consider as ON vs OFF. When "
                "an OUTPUT pins using Normal polarity and is ON it will be 5v, "
                "when OFF it will be 0v, Inverted polarity is the reverse. "
                "INPUT pins using Normal polarity and has a non-zero voltage "
                "it will be treated as ON, a zero voltage will be treated as "
                "OFF, Inverted polarity is the reverse."));

// We factor out the description, and event on/off to a separate group to give
// JMRI a chance to render the make turnout/make sensor buttons.
CDI_GROUP_ENTRY(cfg, IOPinConfig);

CDI_GROUP_ENTRY(output, OutputConfiguration, Name("Output Configuration"),
                Description("This is only valid for Output mode."));
CDI_GROUP_ENTRY(input, InputConfiguration, Name("Input Configuration"),
                Description("This is only valid for Input mode."));

/// reserved space
CDI_GROUP_ENTRY(unused, openlcb::EmptyGroup<5>, Hidden(true));
CDI_GROUP_END();

/// Declares a repeated group of pin configuration data.
using IO_PINS = openlcb::RepeatedGroup<PinConfig, 8>;

/// Defines the main segment in the configuration CDI. This is laid out at
/// origin 128 to give space for the ACDI user data at the beginning.
CDI_GROUP(IoBoard, Segment(openlcb::MemoryConfigDefs::SPACE_CONFIG),
          Offset(128));
/// Each entry declares the name of the current entry, then the type and then
/// optional arguments list.
CDI_GROUP_ENTRY(internal_config, openlcb::InternalConfigData);
CDI_GROUP_ENTRY(gpio, IO_PINS, Name("Input/Output"), RepName("Pin"));
CDI_GROUP_END();

/// This segment is only needed temporarily until there is program code to set
/// the ACDI user data version byte.
CDI_GROUP(VersionSeg, Segment(openlcb::MemoryConfigDefs::SPACE_CONFIG),
    Name("Version information"));
CDI_GROUP_ENTRY(acdi_user_version, openlcb::Uint8ConfigEntry,
    Name("ACDI User Data version"), Description("Set to 2 and do not change."));
CDI_GROUP_END();

/// The main structure of the CDI. ConfigDef is the symbol we use in main.cxx
/// to refer to the configuration defined here.
CDI_GROUP(ConfigDef, MainCdi());
/// Adds the <identification> tag with the values from SNIP_STATIC_DATA above.
CDI_GROUP_ENTRY(ident, openlcb::Identification);
/// Adds an <acdi> tag.
CDI_GROUP_ENTRY(acdi, openlcb::Acdi);
/// Adds a segment for changing the values in the ACDI user-defined
/// space. UserInfoSegment is defined in the system header.
CDI_GROUP_ENTRY(userinfo, openlcb::UserInfoSegment, Name("User Info"));
/// Adds the main configuration segment.
CDI_GROUP_ENTRY(seg, IoBoard, Name("Settings"));
/// Adds the versioning segment.
CDI_GROUP_ENTRY(version, VersionSeg);
CDI_GROUP_END();

} // namespace esp32c3io

#endif // CDI_HXX_