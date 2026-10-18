#pragma once

#include <ros/ros.h>
#include <stdexcept>
#include <string>

// Read into the declared ROS parameter type before assigning to float/enum fields.
template <typename ParameterT, typename ValueT>
void readRosParam(const ros::NodeHandle &node, const std::string &name,
                  ValueT &value, const ParameterT &default_value)
{
  ParameterT parameter;
  if (node.hasParam(name) && !node.getParam(name, parameter))
    throw std::invalid_argument("Invalid parameter type: " + node.resolveName(name));
  if (!node.hasParam(name)) parameter = default_value;
  value = parameter;
}
