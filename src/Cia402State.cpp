#include "Cia402State.h"

namespace cia402 {

State decodeState(const std::uint16_t statusWord)
{
    if ((statusWord & 0x004fU) == 0x0000U) return State::NotReadyToSwitchOn;
    if ((statusWord & 0x004fU) == 0x0040U) return State::SwitchOnDisabled;
    if ((statusWord & 0x006fU) == 0x0021U) return State::ReadyToSwitchOn;
    if ((statusWord & 0x006fU) == 0x0023U) return State::SwitchedOn;
    if ((statusWord & 0x006fU) == 0x0027U) return State::OperationEnabled;
    if ((statusWord & 0x006fU) == 0x0007U) return State::QuickStopActive;
    if ((statusWord & 0x004fU) == 0x000fU) return State::FaultReactionActive;
    if ((statusWord & 0x004fU) == 0x0008U) return State::Fault;
    return State::Unknown;
}

std::string stateName(const State state)
{
    switch (state) {
    case State::NotReadyToSwitchOn: return "Not ready to switch on";
    case State::SwitchOnDisabled: return "Switch on disabled";
    case State::ReadyToSwitchOn: return "Ready to switch on";
    case State::SwitchedOn: return "Switched on";
    case State::OperationEnabled: return "Operation enabled";
    case State::QuickStopActive: return "Quick stop active";
    case State::FaultReactionActive: return "Fault reaction active";
    case State::Fault: return "Fault";
    case State::Unknown: return "Unknown";
    }
    return "Unknown";
}

std::string modeName(const std::int8_t mode)
{
    switch (mode) {
    case -1: return "Manufacturer specific (-1)";
    case 0: return "No mode assigned (0)";
    case 1: return "Profile position (1)";
    case 2: return "Velocity (2)";
    case 3: return "Profile velocity (3)";
    case 4: return "Profile torque (4)";
    case 6: return "Homing (6)";
    case 7: return "Interpolated position (7)";
    case 8: return "Cyclic synchronous position (8)";
    case 9: return "Cyclic synchronous velocity (9)";
    case 10: return "Cyclic synchronous torque (10)";
    default: return "Mode " + std::to_string(mode);
    }
}

} // namespace cia402

