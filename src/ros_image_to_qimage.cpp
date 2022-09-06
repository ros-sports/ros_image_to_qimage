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

#include "ros_image_to_qimage/ros_image_to_qimage.hpp"
#include "cv_bridge/cv_bridge.h"

namespace ros_image_to_qimage
{

QImage Convert(
  const sensor_msgs::msg::Image & msg,
  const cv_bridge::CvtColorForDisplayOptions & options)
{
  cv::Mat conversion_mat_;

  try {
    // Convert image from ros to cv type
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg);
    if (cv_ptr->encoding != sensor_msgs::image_encodings::RGB8) {
      cv_ptr = cv_bridge::cvtColorForDisplay(cv_ptr, "", options);
    }
    conversion_mat_ = cv_ptr->image;
  } catch (cv_bridge::Exception & e) {
    qWarning(
      "ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an "
      "exception was thrown (%s)",
      msg.encoding.c_str(), e.what());
    return QImage{};
  }

  // construct a temporary qimage which doesn't perform a deep copy of the image bytes,
  // then explicitly call copy(), such that a deep copy is performed.
  return QImage(
    conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows,
    conversion_mat_.step[0], QImage::Format_RGB888).copy();
}

}  // namespace ros_image_to_qimage
