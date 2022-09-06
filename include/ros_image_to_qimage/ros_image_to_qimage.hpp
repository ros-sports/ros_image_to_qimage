// Copyright 2021 Kenji Brameld
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

#ifndef ROS_IMAGE_TO_QIMAGE__ROS_IMAGE_TO_QIMAGE_HPP_
#define ROS_IMAGE_TO_QIMAGE__ROS_IMAGE_TO_QIMAGE_HPP_

#include <QImage>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

namespace ros_image_to_qimage
{

/**
 * @brief Converts a ROS image message to a QImage.
 * @details Converts ROS image messages of different encodings to QImage. This function calls
 * cv_bridge::toCvCopy to convert the image to a cv image first. If the cv image is not of type
 * rgb8, it converts the image to an rgb format using cv_bridge::CvtColorForDisplay with
 * \p options . The cv image is then converted to a QImage.
 *
 * @param msg The ROS image message to convert.
 * @param options The options to use when converting a single-channeled image (eg. depth images)
 * @return QImage The QImage representation of the ROS image message.
 */
QImage Convert(
  const sensor_msgs::msg::Image & msg,
  const cv_bridge::CvtColorForDisplayOptions & options = cv_bridge::CvtColorForDisplayOptions());

}  // namespace ros_image_to_qimage

#endif  // ROS_IMAGE_TO_QIMAGE__ROS_IMAGE_TO_QIMAGE_HPP_
