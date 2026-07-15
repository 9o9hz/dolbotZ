/**
 * \file acceleration_type.hpp
 * \mainpage
 *    Index of the acceleration/deceleration types
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef RMD_SDK__ACTUATOR_STATE__ACCELERATION_TYPE
#define RMD_SDK__ACTUATOR_STATE__ACCELERATION_TYPE
#pragma once

#include <cstdint>


namespace rmd_sdk {

  /**\enum AccelerationType
   * \brief
   *    Strongly typed enum for the different acceleration (from initial to maximum speed)/
   *    deceleration (from maximum speed to stop) types
  */
  enum class AccelerationType: std::uint8_t {
    POSITION_PLANNING_ACCELERATION = 0x00,
    POSITION_PLANNING_DECELERATION = 0x01,
    VELOCITY_PLANNING_ACCELERATION = 0x02,
    VELOCITY_PLANNING_DECELERATION = 0x03
  };

}

#endif // RMD_SDK__ACTUATOR_STATE__ACCELERATION_TYPE
