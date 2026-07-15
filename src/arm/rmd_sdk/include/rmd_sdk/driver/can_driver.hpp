/**
 * \file can_driver.hpp
 * \mainpage
 *    Contains the main CAN driver
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef RMD_SDK__DRIVER__CAN_DRIVER
#define RMD_SDK__DRIVER__CAN_DRIVER
#pragma once

#include <string>

#include "rmd_sdk/driver/can_address_offset.hpp"
#include "rmd_sdk/driver/can_node.hpp"


namespace rmd_sdk {

  class ActuatorInterface;

  /**\class CanDriver
   * \brief
   *    CAN driver for commanding several MyActuator RMD actuators
  */
  class CanDriver: public CanNode<CanAddressOffset::request,CanAddressOffset::response> {
    public:
      /**\fn CanDriver
       * \brief
       *    Class constructor
       * 
       * \param[in] ifname
       *    The name of the network interface that should communicated over
      */
      CanDriver(std::string const& ifname)
      : CanNode{ifname} {
        return;
      }

      CanDriver() = delete;
      CanDriver(CanDriver const&) = delete;
      CanDriver& operator = (CanDriver const&) = default;
      CanDriver(CanDriver&&) = default;
      CanDriver& operator = (CanDriver&&) = default;

      friend ActuatorInterface;
  };

}

#endif // RMD_SDK__DRIVER__CAN_DRIVER
