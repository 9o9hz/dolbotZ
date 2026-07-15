/**
 * \file actuator_adaptor.hpp
 * \mainpage
 *    Contains an adaptor for the actuator mock
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef RMD_SDK__TEST__MOCK__ACTUATOR_ADAPTOR
#define RMD_SDK__TEST__MOCK__ACTUATOR_ADAPTOR
#pragma once

#include <cstdint>
#include <string>

#include "rmd_sdk/driver/can_address_offset.hpp"
#include "rmd_sdk/driver/can_node.hpp"
#include "rmd_sdk/protocol/responses.hpp"


namespace rmd_sdk {
  namespace test {

    /**\class ActuatorAdaptor
     * \brief
     *    Counter-part to the actuator and parent class of the mock that can be used for testing the driver
     *    over a (virtual) CAN network interface
    */
    class ActuatorAdaptor: protected rmd_sdk::CanNode<CanAddressOffset::response,CanAddressOffset::request> {
      public:
        /**\fn handleRequest
         * \brief
         *    Read a single CAN frame and handle the request by calling one of the virtual member functions
        */
        void handleRequest();

        /**\fn getVersionDate
         * \brief
         *    Get the version date of the actuator
         *
         * \return
         *    The response that should be sent to the driver
        */
        [[nodiscard]]
        virtual rmd_sdk::GetVersionDateResponse getVersionDate() const = 0;

      protected:
        /**\fn ActuatorAdaptor
         * \brief
         *    Class constructor
         * 
         * \param[in] ifname
         *    The name of the (virtual) CAN network interface that should be used as a loopback device
         * \param[in] actuator_id
         *    The actuator id for the actuator and the actuator mock
        */
        ActuatorAdaptor(std::string const& ifname, std::uint32_t const actuator_id);
        ActuatorAdaptor() = delete;
        ActuatorAdaptor(ActuatorAdaptor const&) = delete;
        ActuatorAdaptor& operator = (ActuatorAdaptor const&) = default;
        ActuatorAdaptor(ActuatorAdaptor&&) = default;
        ActuatorAdaptor& operator = (ActuatorAdaptor&&) = default;

        std::uint32_t actuator_id_;
    };

  }
}

#endif // RMD_SDK__TEST__MOCK__ACTUATOR_ADAPTOR
