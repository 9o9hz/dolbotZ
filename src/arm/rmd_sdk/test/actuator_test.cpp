/**
 * \file actuator_test.cpp
 * \mainpage
 *    Test the communication between the actuator class and the actuator mock
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rmd_sdk/protocol/responses.hpp"
#include "mock/actuator_actuator_mock_test.hpp"


namespace rmd_sdk {
  namespace test {

    TEST_F(ActuatorActuatorMockTest, getVersionDate) {
      rmd_sdk::GetVersionDateResponse const response {{0xB2, 0x00, 0x00, 0x00, 0x2E, 0x89, 0x34, 0x01}};
      EXPECT_CALL(actuator_mock_, getVersionDate).WillOnce(::testing::Return(response));
      auto const version {actuator_.getVersionDate()};
      EXPECT_EQ(version, 20220206);
    }

  }
}
