/**
 * \file conversions.hpp
 * \mainpage
 *    Contains functions for converting physical units
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef RMD_HARDWARE_INTERFACE__CONVERSIONS
#define RMD_HARDWARE_INTERFACE__CONVERSIONS
#pragma once

#include <cmath>


namespace rmd_hardware_interface {

  /**\fn degToRad
   * \brief
   *    Convert an angle in degree to radians
   * 
   * \param[in] degrees
   *    The angle in degree
   * \return
   *    The angle in radians
  */
  constexpr double degToRad(double const degrees) noexcept {
    return degrees*(M_PI/180.0);
  }

  /**\fn radToDeg
   * \brief
   *    Convert an angle in radians to degrees
   * 
   * \param[in] radians
   *    The angle in radians
   * \return
   *    The angle in degree
  */
  constexpr double radToDeg(double const radians) noexcept {
    return radians*(180.0/M_PI);
  }

  /**\fn currentToTorque
   * \brief
   *    Convert a given current to a torque by assuming a linear relationship
   * 
   * \param[in] current
   *    The current in Ampere (A) that should be converted to a equivalent torque
   * \param[in] torque_constant
   *    The torque constant in Newton meter per Ampere (Nm/A)
   * \return
   *    The resulting torque in Newton meter (Nm)
  */
  constexpr double currentToTorque(double const current, double const torque_constant) noexcept {
    return current*torque_constant;
  }

  /**\fn torqueToCurrent
   * \brief
   *    Convert a given torque to a current by assuming a linear relationship
   * 
   * \param[in] torque
   *    The torque in Newton meter (Nm) that should be converted to a equivalent current
   * \param[in] torque_constant
   *    The torque constant in Newton meter per Ampere (Nm/A)
   * \return
   *    The resulting current in Ampere (A)
  */
  constexpr double torqueToCurrent(double const torque, double const torque_constant) noexcept {
    return torque/torque_constant;
  }

}  // namespace rmd_hardware_interface

#endif  // RMD_HARDWARE_INTERFACE__CONVERSIONS
