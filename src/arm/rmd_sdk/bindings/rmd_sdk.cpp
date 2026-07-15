/**
 * \file rmd_sdk.cpp
 * \mainpage
 *    Python bindings for C++ library
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#include <cstdint>
#include <string>
#include <sstream>
#include <tuple>

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rmd_sdk/actuator_state/acceleration_type.hpp"
#include "rmd_sdk/actuator_state/can_baud_rate.hpp"
#include "rmd_sdk/actuator_state/control_mode.hpp"
#include "rmd_sdk/actuator_state/error_code.hpp"
#include "rmd_sdk/actuator_state/feedback.hpp"
#include "rmd_sdk/actuator_state/gains.hpp"
#include "rmd_sdk/actuator_state/motor_status_1.hpp"
#include "rmd_sdk/actuator_state/motor_status_2.hpp"
#include "rmd_sdk/actuator_state/motor_status_3.hpp"
#include "rmd_sdk/can/exceptions.hpp"
#include "rmd_sdk/can/frame.hpp"
#include "rmd_sdk/can/node.hpp"
#include "rmd_sdk/driver/can_driver.hpp"
#include "rmd_sdk/driver/driver.hpp"
#include "rmd_sdk/actuator_constants.hpp"
#include "rmd_sdk/actuator_interface.hpp"
#include "rmd_sdk/exceptions.hpp"
#include "rmd_sdk/io.hpp"


namespace rmd_sdk {
  namespace bindings {

    /**\fn declareActuator
     * \brief
     *    Helper function for declaring the actuator constants for a given actuator
     * 
     * \tparam T
     *    The class containing the actuator constants
     * \param[in] m
     *    Pybind11 module that the actuator constants should be declared into
     * \param[in] class_name
     *    Class name of the corresponding Python bindings
    */
    template<typename T>
    void declareActuator(pybind11::module& m, std::string const& class_name) {
      pybind11::class_<T>(m, class_name.c_str())
        .def_readonly_static("reducer_ratio", &T::reducer_ratio)
        .def_readonly_static("rated_speed", &T::rated_speed)
        .def_readonly_static("rated_current", &T::rated_current)
        .def_readonly_static("rated_power", &T::rated_power)
        .def_readonly_static("rated_torque", &T::rated_torque)
        .def_readonly_static("torque_constant", &T::torque_constant)
        .def_readonly_static("rotor_inertia", &T::rotor_inertia);
      return;
    }

  }
}

PYBIND11_MODULE(rmd_sdk_py, m) {

  m.doc() = "Python bindings for MyActuator RMD-X actuator series";
  pybind11::class_<rmd_sdk::Driver>(m, "Driver");
  pybind11::class_<rmd_sdk::CanDriver, rmd_sdk::Driver>(m, "CanDriver")
    .def(pybind11::init<std::string const&>());
  pybind11::class_<rmd_sdk::ActuatorInterface>(m, "ActuatorInterface")
    .def(pybind11::init<rmd_sdk::Driver&, std::uint32_t>())
    .def("getAcceleration", &rmd_sdk::ActuatorInterface::getAcceleration)
    .def("getCanId", &rmd_sdk::ActuatorInterface::getCanId)
    .def("getControllerGains", &rmd_sdk::ActuatorInterface::getControllerGains)
    .def("getControlMode", &rmd_sdk::ActuatorInterface::getControlMode)
    .def("getMotorModel", &rmd_sdk::ActuatorInterface::getMotorModel)
    .def("getMotorPower", &rmd_sdk::ActuatorInterface::getMotorPower)
    .def("getMotorStatus1", &rmd_sdk::ActuatorInterface::getMotorStatus1)
    .def("getMotorStatus2", &rmd_sdk::ActuatorInterface::getMotorStatus2)
    .def("getMotorStatus3", &rmd_sdk::ActuatorInterface::getMotorStatus3)
    .def("getMultiTurnAngle", &rmd_sdk::ActuatorInterface::getMultiTurnAngle)
    .def("getMultiTurnEncoderPosition", &rmd_sdk::ActuatorInterface::getMultiTurnEncoderPosition)
    .def("getMultiTurnEncoderOriginalPosition", &rmd_sdk::ActuatorInterface::getMultiTurnEncoderOriginalPosition)
    .def("getMultiTurnEncoderZeroOffset", &rmd_sdk::ActuatorInterface::getMultiTurnEncoderZeroOffset)
    .def("getRuntime", &rmd_sdk::ActuatorInterface::getRuntime)
    .def("getSingleTurnAngle", &rmd_sdk::ActuatorInterface::getSingleTurnAngle)
    .def("getSingleTurnEncoderPosition", &rmd_sdk::ActuatorInterface::getSingleTurnEncoderPosition)
    .def("getVersionDate", &rmd_sdk::ActuatorInterface::getVersionDate)
    .def("lockBrake", &rmd_sdk::ActuatorInterface::lockBrake)
    .def("releaseBrake", &rmd_sdk::ActuatorInterface::releaseBrake)
    .def("reset", &rmd_sdk::ActuatorInterface::reset)
    .def("sendCurrentSetpoint", &rmd_sdk::ActuatorInterface::sendCurrentSetpoint)
    .def("sendPositionAbsoluteSetpoint", &rmd_sdk::ActuatorInterface::sendPositionAbsoluteSetpoint)
    .def("sendTorqueSetpoint", &rmd_sdk::ActuatorInterface::sendTorqueSetpoint)
    .def("sendVelocitySetpoint", &rmd_sdk::ActuatorInterface::sendVelocitySetpoint)
    .def("setAcceleration", &rmd_sdk::ActuatorInterface::setAcceleration)
    .def("setCanBaudRate", &rmd_sdk::ActuatorInterface::setCanBaudRate)
    .def("setCanId", &rmd_sdk::ActuatorInterface::setCanId)
    .def("setControllerGains", &rmd_sdk::ActuatorInterface::setControllerGains)
    .def("setCurrentPositionAsEncoderZero", &rmd_sdk::ActuatorInterface::setCurrentPositionAsEncoderZero)
    .def("setEncoderZero", &rmd_sdk::ActuatorInterface::setEncoderZero)
    .def("setTimeout", &rmd_sdk::ActuatorInterface::setTimeout)
    .def("shutdownMotor", &rmd_sdk::ActuatorInterface::shutdownMotor)
    .def("stopMotor", &rmd_sdk::ActuatorInterface::stopMotor);
  pybind11::register_exception<rmd_sdk::Exception>(m, "ActuatorException");
  pybind11::register_exception<rmd_sdk::ProtocolException>(m, "ProtocolException");
  pybind11::register_exception<rmd_sdk::ValueRangeException>(m, "ValueRangeException");

  auto m_actuator_state = m.def_submodule("actuator_state", "Submodule for actuator state structures");
  pybind11::enum_<rmd_sdk::AccelerationType>(m_actuator_state, "AccelerationType")
    .value("POSITION_PLANNING_ACCELERATION", rmd_sdk::AccelerationType::POSITION_PLANNING_ACCELERATION)
    .value("POSITION_PLANNING_DECELERATION", rmd_sdk::AccelerationType::POSITION_PLANNING_DECELERATION)
    .value("VELOCITY_PLANNING_ACCELERATION", rmd_sdk::AccelerationType::VELOCITY_PLANNING_ACCELERATION)
    .value("VELOCITY_PLANNING_DECELERATION", rmd_sdk::AccelerationType::VELOCITY_PLANNING_DECELERATION);
  pybind11::enum_<rmd_sdk::CanBaudRate>(m_actuator_state, "CanBaudRate")
    .value("KBPS500", rmd_sdk::CanBaudRate::KBPS500)
    .value("MBPS1", rmd_sdk::CanBaudRate::MBPS1);
  pybind11::enum_<rmd_sdk::ControlMode>(m_actuator_state, "ControlMode")
    .value("NONE", rmd_sdk::ControlMode::NONE)
    .value("CURRENT", rmd_sdk::ControlMode::CURRENT)
    .value("VELOCITY", rmd_sdk::ControlMode::VELOCITY)
    .value("POSITION", rmd_sdk::ControlMode::POSITION);
  pybind11::enum_<rmd_sdk::ErrorCode>(m_actuator_state, "ErrorCode")
    .value("NO_ERROR", rmd_sdk::ErrorCode::NO_ERROR)
    .value("MOTOR_STALL", rmd_sdk::ErrorCode::MOTOR_STALL)
    .value("LOW_VOLTAGE", rmd_sdk::ErrorCode::LOW_VOLTAGE)
    .value("OVERVOLTAGE", rmd_sdk::ErrorCode::OVERVOLTAGE)
    .value("OVERCURRENT", rmd_sdk::ErrorCode::OVERCURRENT)
    .value("POWER_OVERRUN", rmd_sdk::ErrorCode::POWER_OVERRUN)
    .value("SPEEDING", rmd_sdk::ErrorCode::SPEEDING)
    .value("UNSPECIFIED_1", rmd_sdk::ErrorCode::UNSPECIFIED_1)
    .value("UNSPECIFIED_2", rmd_sdk::ErrorCode::UNSPECIFIED_2)
    .value("UNSPECIFIED_3", rmd_sdk::ErrorCode::UNSPECIFIED_3)
    .value("OVERTEMPERATURE", rmd_sdk::ErrorCode::OVERTEMPERATURE)
    .value("ENCODER_CALIBRATION_ERROR", rmd_sdk::ErrorCode::ENCODER_CALIBRATION_ERROR);
  pybind11::class_<rmd_sdk::Gains>(m_actuator_state, "Gains")
    .def(pybind11::init<rmd_sdk::PiGains const&, rmd_sdk::PiGains const&, rmd_sdk::PiGains const&>())
    .def(pybind11::init<std::uint8_t const, std::uint8_t const, std::uint8_t const, std::uint8_t const, std::uint8_t const, std::uint8_t const>())
    .def_readwrite("current", &rmd_sdk::Gains::current)
    .def_readwrite("speed", &rmd_sdk::Gains::speed)
    .def_readwrite("position", &rmd_sdk::Gains::position)
    .def("__repr__", [](rmd_sdk::Gains const& gains) -> std::string { 
      std::ostringstream ss {};
      ss << gains;
      return ss.str();
    });
  pybind11::class_<rmd_sdk::MotorStatus1>(m_actuator_state, "MotorStatus1")
    .def(pybind11::init<int const, bool const, float const, rmd_sdk::ErrorCode const>())
    .def_readonly("temperature", &rmd_sdk::MotorStatus1::temperature)
    .def_readonly("is_brake_released", &rmd_sdk::MotorStatus1::is_brake_released)
    .def_readonly("voltage", &rmd_sdk::MotorStatus1::voltage)
    .def_readonly("error_code", &rmd_sdk::MotorStatus1::error_code)
    .def("__repr__", [](rmd_sdk::MotorStatus1 const& motor_status) -> std::string { 
      std::ostringstream ss {};
      ss << motor_status;
      return ss.str();
    });
  pybind11::class_<rmd_sdk::MotorStatus2>(m_actuator_state, "MotorStatus2")
    .def(pybind11::init<int const, float const, float const, float const>())
    .def_readonly("temperature", &rmd_sdk::MotorStatus2::temperature)
    .def_readonly("current", &rmd_sdk::MotorStatus2::current)
    .def_readonly("shaft_speed", &rmd_sdk::MotorStatus2::shaft_speed)
    .def_readonly("shaft_angle", &rmd_sdk::MotorStatus2::shaft_angle)
    .def("__repr__", [](rmd_sdk::MotorStatus2 const& motor_status) -> std::string { 
      std::ostringstream ss {};
      ss << motor_status;
      return ss.str();
    });
  pybind11::class_<rmd_sdk::MotorStatus3>(m_actuator_state, "MotorStatus3")
    .def(pybind11::init<int const, float const, float const, float const>())
    .def_readonly("temperature", &rmd_sdk::MotorStatus3::temperature)
    .def_readonly("current_phase_a", &rmd_sdk::MotorStatus3::current_phase_a)
    .def_readonly("current_phase_b", &rmd_sdk::MotorStatus3::current_phase_b)
    .def_readonly("current_phase_c", &rmd_sdk::MotorStatus3::current_phase_c)
    .def("__repr__", [](rmd_sdk::MotorStatus3 const& motor_status) -> std::string { 
      std::ostringstream ss {};
      ss << motor_status;
      return ss.str();
    });
  pybind11::class_<rmd_sdk::PiGains>(m_actuator_state, "PiGains")
    .def(pybind11::init<std::uint8_t const, std::uint8_t const>())
    .def_readwrite("kp", &rmd_sdk::PiGains::kp)
    .def_readwrite("ki", &rmd_sdk::PiGains::ki)
    .def("__repr__", [](rmd_sdk::PiGains const& pi_gains) -> std::string { 
      std::ostringstream ss {};
      ss << pi_gains;
      return ss.str();
    });

  auto m_can = m.def_submodule("can", "Submodule for basic CAN communication");
  pybind11::class_<rmd_sdk::can::Frame>(m_can, "Frame")
    .def(pybind11::init<std::uint32_t const, std::array<std::uint8_t,8> const&>())
    .def("getId", &rmd_sdk::can::Frame::getId)
    .def("getData", &rmd_sdk::can::Frame::getData);
  pybind11::class_<rmd_sdk::can::Node>(m_can, "Node")
    .def(pybind11::init<std::string const&>())
    .def("setRecvFilter", &rmd_sdk::can::Node::setRecvFilter)
    .def("read", &rmd_sdk::can::Node::read)
    .def("write", pybind11::overload_cast<rmd_sdk::can::Frame const&>(&rmd_sdk::can::Node::write));
  pybind11::register_exception<rmd_sdk::can::SocketException>(m_can, "SocketException");
  pybind11::register_exception<rmd_sdk::can::Exception>(m_can, "CanException");
  pybind11::register_exception<rmd_sdk::can::TxTimeoutError>(m_can, "TxTimeoutError");
  pybind11::register_exception<rmd_sdk::can::LostArbitrationError>(m_can, "LostArbitrationError");
  pybind11::register_exception<rmd_sdk::can::ControllerProblemError>(m_can, "ControllerProblemError");
  pybind11::register_exception<rmd_sdk::can::ProtocolViolationError>(m_can, "ProtocolViolationError");
  pybind11::register_exception<rmd_sdk::can::TransceiverStatusError>(m_can, "TransceiverStatusError");
  pybind11::register_exception<rmd_sdk::can::NoAcknowledgeError>(m_can, "NoAcknowledgeError");
  pybind11::register_exception<rmd_sdk::can::BusOffError>(m_can, "BusOffError");
  pybind11::register_exception<rmd_sdk::can::BusError>(m_can, "BusError");
  pybind11::register_exception<rmd_sdk::can::ControllerRestartedError>(m_can, "ControllerRestartedError");

  auto m_actuator_constants = m.def_submodule("actuator_constants", "Submodule for actuator constants");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X4V2>(m_actuator_constants,     "X4V2");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X4V3>(m_actuator_constants,     "X4V3");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X4_3>(m_actuator_constants,     "X4_3");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X4_24>(m_actuator_constants,    "X4_24");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X6V2>(m_actuator_constants,     "X6V2");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X6S2V2>(m_actuator_constants,   "X6S2V2");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X6V3>(m_actuator_constants,     "X6V3");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X6_7>(m_actuator_constants,     "X6_7");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X6_8>(m_actuator_constants,     "X6_8");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X6_40>(m_actuator_constants,    "X6_40");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X8V2>(m_actuator_constants,     "X8V2");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X8ProV2>(m_actuator_constants,  "X8ProV2");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X8S2V3>(m_actuator_constants,   "X8S2V3");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X8HV3>(m_actuator_constants,    "X8HV3");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X8ProHV3>(m_actuator_constants, "X8ProHV3");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X8_20>(m_actuator_constants,    "X8_20");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X8_25>(m_actuator_constants,    "X8_25");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X8_60>(m_actuator_constants,    "X8_60");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X8_90>(m_actuator_constants,    "X8_90");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X10V3>(m_actuator_constants,    "X10V3");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X10S2V3>(m_actuator_constants,  "X10S2V3");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X10_40>(m_actuator_constants,   "X10_40");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X10_100>(m_actuator_constants,  "X10_100");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X12_150>(m_actuator_constants,  "X12_150");
  rmd_sdk::bindings::declareActuator<rmd_sdk::X15_400>(m_actuator_constants,  "X15_400");

}
