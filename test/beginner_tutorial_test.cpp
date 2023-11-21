/**
 * @file beginner_tutorial_test.cpp
 * @author Vinay Lanka (120417665)
 * @brief
 * @version 0.1
 * @date 2023-11-20
 *
 * @copyright Copyright (c) 2023 Vinay Lanka
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 *
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <beginner_tutorials/srv/change_str.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Testing fixture for the level 2 ROS Node Test
 *
 */
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    bool retVal =
        StartROSExec("beginner_tutorials", "transform_publisher", "talker");
    ASSERT_TRUE(retVal);
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    bool retVal = StopROSExec();
    ASSERT_TRUE(retVal);
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;

  bool StartROSExec(const char* pkg_name, const char* node_name,
                    const char* exec_name) {
    // build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name
           << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info "
               << "/" << node_name << " > /dev/null 2> /dev/null";
    char execName[16];
    snprintf(execName, sizeof(execName), "%s",
             exec_name);  // pkill uses exec name <= 15 char only
    killCmd_ss << "pkill --signal SIGINT " << execName
               << " > /dev/null 2> /dev/null";

    // First kill the ros2 node, in case it's still running.
    StopROSExec();

    // Start a ros2 node and wait for it to get ready:
    int retVal = system(cmd_ss.str().c_str());
    if (retVal != 0) return false;

    retVal = -1;
    while (retVal != 0) {
      retVal = system(cmdInfo_ss.str().c_str());
      sleep(1);
    }
    return true;
  }

  bool StopROSExec() {
    if (killCmd_ss.str().empty()) return true;

    int retVal = system(killCmd_ss.str().c_str());
    return retVal == 0;
  }
};

/**
 * @brief Construct a new test f object to test the change string functionality
 *
 */
TEST_F(TaskPlanningFixture, ChangeStrTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  using CLIENT = rclcpp::Client<beginner_tutorials::srv::ChangeStr>::SharedPtr;

  CLIENT client =
      node_->create_client<beginner_tutorials::srv::ChangeStr>("change_string");

  auto request =
      std::make_shared<beginner_tutorials::srv::ChangeStr::Request>();
  request->new_string = "test string";

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Result - " <<
    // result.get()->string_change_status);
    ASSERT_EQ(result.get()->string_change_status, "test string");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service add_two_ints");
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
