// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// gtest
#include <rclcpp/node.hpp>
#include "gtest/gtest.h"

// romea
// clang-format off
#include "test_node_publisher_utils.hpp"
// clang-format on
#include "romea_common_utils/log/topic_logger.hpp"

class TestTopicLogger : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override { node = std::make_shared<rclcpp::Node>("test_topic_logger"); }

  void SleedpAndSpinSome()
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin_some(node);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestTopicLogger, basic)
{
  using TopicLogger = romea::ros2::TopicLogger<rclcpp::Node>;
  TopicLogger logger(node, "log", 1, {}, true);
  Subscription<TopicLogger::DataLoggerMsg> sub(node, "log");

  logger.addEntry("cheval", 3.1415);
  logger.addEntry("mouton", 5);
  logger.writeRow();

  SleedpAndSpinSome();

  {
    auto msg = sub.get_received_data();
    ASSERT_EQ(msg.names.size(), 2);
    ASSERT_EQ(msg.values.size(), 2);

    EXPECT_EQ(msg.names.front(), "cheval");
    EXPECT_EQ(msg.names.back(), "mouton");
    EXPECT_DOUBLE_EQ(msg.values.front(), 3.1415);
    EXPECT_DOUBLE_EQ(msg.values.back(), 5);
  }

  logger.addEntry("cheval", 11.1);
  logger.addEntry("mouton", 256.0);
  logger.writeRow();

  SleedpAndSpinSome();

  {
    auto msg = sub.get_received_data();
    ASSERT_EQ(msg.names.size(), 2);
    ASSERT_EQ(msg.values.size(), 2);

    EXPECT_EQ(msg.names.front(), "cheval");
    EXPECT_EQ(msg.names.back(), "mouton");
    EXPECT_DOUBLE_EQ(msg.values.front(), 11.1);
    EXPECT_DOUBLE_EQ(msg.values.back(), 256.0);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
