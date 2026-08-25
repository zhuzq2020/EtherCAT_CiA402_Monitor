#pragma once

#include <cstdint>
#include <string>

namespace cia402 {

enum class State {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
    Unknown
};

State decodeState(std::uint16_t statusWord);
std::string stateName(State state);
std::string modeName(std::int8_t mode);

} // namespace cia402

