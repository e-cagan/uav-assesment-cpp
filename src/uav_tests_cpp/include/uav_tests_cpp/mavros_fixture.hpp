#pragma once
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

class MavrosFixture : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("uav_tests_node");
  }
  void TearDown() override {
    node_.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
  }
  rclcpp::Node::SharedPtr node_;
};
