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
 * @details Converts ROS image messages of different encodings to QImage. If a 3-channel image is
 * passed in \p msg, (eg. an rgb color image), it will be converted to a QImage directly.
 * If a single channel image is passed in \p msg, (eg. a depth image), this function calls
 * cv_bridge::CvtColorForDisplay to convert the image to an rgb format with the specified
 * \p options, which is then converted to a QImage.
 *
 * @param msg The ROS image message to convert.
 * @param options The options to use when converting a single-channeled image (eg. depth images)
 * @return QImage The QImage representation of the ROS image message.
 */
QImage Convert(
  const sensor_msgs::msg::Image & msg,
  const cv_bridge::CvtColorForDisplayOptions & options = cv_bridge::CvtColorForDisplayOptions());

/**
 * @brief Returns whether an encoding is supported by this library
 *
 * @param encoding The image encoding type
 * @return true if the encoding is supported, false otherwise
 */
bool IsEncodingSupported(std::string encoding);

}  // namespace ros_image_to_qimage

#endif  // ROS_IMAGE_TO_QIMAGE__ROS_IMAGE_TO_QIMAGE_HPP_
