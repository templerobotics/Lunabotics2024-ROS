#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "core.hpp"

class TestTeleopControl : public ::testing::Test 
{
protected:
  void SetUp() override 
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_teleop_control");
  }

  void TearDown() override 
  {
    node.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestTeleopControl, TestNodeCreation) 
{
  ASSERT_NE(node, nullptr);
  std::string node_name = node->get_name();
  ASSERT_EQ(node_name, "test_teleop_control");
}

int main(int argc, char ** argv) 
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}