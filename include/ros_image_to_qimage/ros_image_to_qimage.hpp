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
#include "sensor_msgs/msg/image.hpp"

namespace ros_image_to_qimage
{

QImage convert(const sensor_msgs::msg::Image::ConstSharedPtr msg);

}  // namespace ros_image_to_qimage

#endif  // ROS_IMAGE_TO_QIMAGE__ROS_IMAGE_TO_QIMAGE_HPP_
