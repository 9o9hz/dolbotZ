/**
 * \file utilities.hpp
 * \mainpage
 *    Contains utility functions for various Linux structs
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef RMD_SDK__CAN__UTILITIES
#define RMD_SDK__CAN__UTILITIES
#pragma once

#include <chrono>
#include <ostream>
#include <ratio>

#include <linux/can.h>
#include <sys/time.h>


/**\fn operator <<
 * \brief
 *    Output stream operator for a Linux SocketCAN CAN frame
 *
 * \param[in,out] os
 *    The output stream that should be written to
 * \param[in] frame
 *    The CAN frame that should be written to the output stream
 * \return
 *    The output stream containing information about the CAN frame
*/
std::ostream& operator << (std::ostream& os, struct ::can_frame const& frame) noexcept;

namespace rmd_sdk {

  /**\fn toTimeval
   * \brief
   *    Function for converting any std::chrono::duration to a Linux timeval struct
   *
   * \tparam Rep
   *    Arithmetic type representing the number of ticks
   * \tparam Period
   *    An std::ratio representing the tick period
   * \param[in] duration
   *    The duration that should be converted to a Linux timeval struct
   * \return
   *    The duration as a Linux timeval struct
  */
  template <class Rep, class Period>
  [[nodiscard]]
  constexpr struct ::timeval toTimeval(std::chrono::duration<Rep, Period> const& duration) noexcept {
    auto const usec {std::chrono::duration_cast<std::chrono::duration<Rep, std::micro>>(duration)};
    struct ::timeval tv {};
    tv.tv_sec = static_cast<::time_t>(usec.count()/std::micro::den);
    tv.tv_usec = static_cast<long int>(usec.count())%std::micro::den;
    return tv;
  }

}

#endif // RMD_SDK__CAN__UTILITIES
