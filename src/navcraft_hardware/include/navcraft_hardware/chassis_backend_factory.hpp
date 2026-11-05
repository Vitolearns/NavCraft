#pragma once

#include <memory>
#include <string>

#include "navcraft_hardware/chassis_backend.hpp"

namespace navcraft_hardware {

std::unique_ptr<ChassisBackend> CreateChassisBackend(
    const std::string& backend, const ChassisBackendConfig& config);

}  // namespace navcraft_hardware
