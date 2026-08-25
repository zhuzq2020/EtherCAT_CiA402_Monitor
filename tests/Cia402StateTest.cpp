#include "Cia402State.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>

int main()
{
    using cia402::State;
    const std::array<std::pair<std::uint16_t, State>, 8> cases{{
        {0x0000, State::NotReadyToSwitchOn},
        {0x0040, State::SwitchOnDisabled},
        {0x0021, State::ReadyToSwitchOn},
        {0x0023, State::SwitchedOn},
        {0x0027, State::OperationEnabled},
        {0x0007, State::QuickStopActive},
        {0x000f, State::FaultReactionActive},
        {0x0008, State::Fault},
    }};

    for (const auto &[word, expected] : cases) {
        assert(cia402::decodeState(word) == expected);
    }
    assert(cia402::decodeState(0xffff) == State::Unknown);
    assert(cia402::modeName(8).find("Cyclic synchronous position") != std::string::npos);
    std::cout << "CiA402 state decoder: all tests passed\n";
    return 0;
}

