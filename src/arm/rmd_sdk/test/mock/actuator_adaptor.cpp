#include "actuator_adaptor.hpp"

#include <array>
#include <cstdint>
#include <string>

#include "rmd_sdk/can/frame.hpp"
#include "rmd_sdk/driver/can_driver.hpp"
#include "rmd_sdk/protocol/command_type.hpp"
#include "rmd_sdk/protocol/responses.hpp"
#include "rmd_sdk/exceptions.hpp"


namespace rmd_sdk {
  namespace test {

    void ActuatorAdaptor::handleRequest() {
      can::Frame const frame {read()};
      std::array<std::uint8_t,8> const data {frame.getData()};
      
      if (data[0] == rmd_sdk::CommandType::READ_SYSTEM_SOFTWARE_VERSION_DATE) {
        rmd_sdk::GetVersionDateResponse const response {getVersionDate()};
        send(response, actuator_id_);
      } else {
        throw rmd_sdk::Exception("Unrecognized request");
      }
      return;
    }

    ActuatorAdaptor::ActuatorAdaptor(std::string const& ifname, std::uint32_t const actuator_id)
    : CanNode{ifname}, actuator_id_{actuator_id} {
      this->addId(actuator_id_);
      return;
    }

  }
}
