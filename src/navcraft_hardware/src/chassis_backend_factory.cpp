#include "navcraft_hardware/chassis_backend_factory.hpp"

#include "navcraft_hardware/mock_backend.hpp"
#include "navcraft_hardware/serial_backend.hpp"
#include "navcraft_hardware/udp_backend.hpp"

namespace navcraft_hardware {

std::unique_ptr<ChassisBackend> CreateChassisBackend(
    const std::string& backend, const ChassisBackendConfig& config) {
  if (backend == "serial") {
    return std::make_unique<SerialBackend>(config);
  }
  if (backend == "udp") {
    return std::make_unique<UdpBackend>(config);
  }
  return std::make_unique<MockBackend>();
}

}  // namespace navcraft_hardware
