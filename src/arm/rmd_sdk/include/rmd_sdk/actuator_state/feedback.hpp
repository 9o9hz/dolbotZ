/**
 * \file feedback.hpp
 * \mainpage
 *    Contains the struct for closed-loop control feedback
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef RMD_SDK__ACTUATOR_STATE__FEEDBACK
#define RMD_SDK__ACTUATOR_STATE__FEEDBACK
#pragma once

#include "rmd_sdk/actuator_state/motor_status_2.hpp"


namespace rmd_sdk {

  // The feedback struct for any closed-loop control corresponds to motor status 2
  using Feedback = MotorStatus2;

}

#endif // RMD_SDK__ACTUATOR_STATE__FEEDBACK
