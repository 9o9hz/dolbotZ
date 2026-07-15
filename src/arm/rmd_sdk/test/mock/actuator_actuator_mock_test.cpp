#include "actuator_actuator_mock_test.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include "rmd_sdk/driver/can_driver.hpp"
#include "actuator_mock.hpp"


namespace rmd_sdk {
  namespace test {

    ActuatorActuatorMockTest::ActuatorActuatorMockTest(std::string const& ifname, std::uint32_t const actuator_id)
    : driver_{ifname}, actuator_{driver_, actuator_id}, actuator_mock_{ifname, actuator_id}, mock_thread_{} {
      return;
    }

    void ActuatorActuatorMockTest::SetUp() {
      mock_thread_ = std::thread(&rmd_sdk::test::ActuatorMock::handleRequest, std::ref(actuator_mock_));
      return;
    }

    void ActuatorActuatorMockTest::TearDown() {
      if (mock_thread_.joinable()) {
        mock_thread_.join();
      }
      return;
    }

  }
}
