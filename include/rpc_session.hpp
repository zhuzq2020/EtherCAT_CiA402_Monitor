#pragma once

#include "arm_controller.hpp"

namespace robot_maint {

// Runs a single-bus-session line-protocol RPC loop over stdin/stdout
// so Python workflow scripts (backup_sii.py, write_alias.py, configure_pdo.py)
// can orchestrate multi-step CoE SDO and ESC/EEPROM operations over one open socket.
int run_rpc_session(ArmController& controller);

} // namespace robot_maint
