//
// Created by yenkn on 2022/7/18.
//
#include "nokov_localization/subscriber.h"
#include "nlohmann/json.hpp"

NokovLocalizationSubscriber::NokovLocalizationSubscriber(ros::NodeHandle &node) {
  info_sub_ = node.subscribe("/nokov_info", 1, &NokovLocalizationSubscriber::InfoCallback, this);
}

void NokovLocalizationSubscriber::InfoCallback(const std_msgs::StringConstPtr &msg) {
  nlohmann::json obj;
  try {
    obj = nlohmann::json::parse(msg->data);
  } catch(const std::exception &ex) {
    ROS_ERROR("error parsing json: %s", ex.what());
    return;
  }

  if(obj.contains("boundary")) {
    // TODO:
  }

}