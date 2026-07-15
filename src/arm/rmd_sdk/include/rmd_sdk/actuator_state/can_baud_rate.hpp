/**
 * \file can_baud_rate.hpp
 * \mainpage
 *    Communication Baud rate of the CAN bus
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef RMD_SDK__ACTUATOR_STATE__CAN_BAUD_RATE
#define RMD_SDK__ACTUATOR_STATE__CAN_BAUD_RATE
#pragma once

#include <cstdint>


namespace rmd_sdk {

  /**\enum CanBaudRate
  * \brief
  *    Communication Baud rate of the CAN bus
  */
  enum class CanBaudRate: std::uint8_t {
    KBPS500 = 0,
    MBPS1 = 1
  };

}

#endif // RMD_SDK__ACTUATOR_STATE__CAN_BAUD_RATE
