#pragma once

#include "ros_params.h"
#include <vikit/pinhole_camera.h>
#include <vikit/equidistant_camera.h>
#include <cmath>
#include <memory>

namespace sa_livo {
inline bool loadCamera(const ros::NodeHandle &node, const std::string &name,
                       vk::AbstractCamera *&camera)
{
  ros::NodeHandle params(node, name);
  auto required = [&params](const std::string &key) {
    double value;
    if (!params.getParam(key, value) || !std::isfinite(value))
      throw std::invalid_argument("Missing or invalid camera parameter: " + params.resolveName(key));
    return value;
  };
  std::string model;
  if (!params.getParam("model", model))
    throw std::invalid_argument("Missing camera model: " + params.getNamespace());
  const double width = required("width"), height = required("height");
  const double fx = required("fx"), fy = required("fy");
  const double cx = required("cx"), cy = required("cy");
  double scale;
  readRosParam<double>(params, "scale", scale, 1.0);
  if (width <= 0 || height <= 0 || fx <= 0 || fy <= 0 ||
      !std::isfinite(scale) || scale <= 0 || width * scale < 1 || height * scale < 1)
    throw std::invalid_argument("Invalid camera dimensions, focal length or scale: " + name);
  auto distortion = [&params](const std::string &key) {
    double value;
    readRosParam<double>(params, key, value, 0.0);
    if (!std::isfinite(value)) throw std::invalid_argument("Invalid distortion: " + key);
    return value;
  };
  std::unique_ptr<vk::AbstractCamera> result;
  if (model == "Pinhole")
    result.reset(new vk::PinholeCamera(width, height, scale, fx, fy, cx, cy,
        distortion("d0"), distortion("d1"), distortion("d2"), distortion("d3")));
  else if (model == "EquidistantCamera")
    result.reset(new vk::EquidistantCamera(width, height, scale, fx, fy, cx, cy,
        distortion("k1"), distortion("k2"), distortion("k3"), distortion("k4")));
  else
    throw std::invalid_argument("Unsupported camera model: " + model);
  camera = result.release();
  return true;
}
} // namespace sa_livo
