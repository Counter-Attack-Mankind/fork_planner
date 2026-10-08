//
// Created by yenkn on 2022/7/18.
//
#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <functional>
#include <vector>

using BoundaryCallback = std::function<void(const std::array<double, 4> &msg)>;

class NokovLocalizationSubscriber {
public:
  explicit NokovLocalizationSubscriber(ros::NodeHandle &node);

private:
  ros::Subscriber info_sub_;

  void InfoCallback(const std_msgs::StringConstPtr &msg);
};