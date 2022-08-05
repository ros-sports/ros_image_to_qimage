// Copyright 2022 Kenji Brameld
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

#include "gtest/gtest.h"
#include "ros_image_to_qimage/ros_image_to_qimage.hpp"
#include "sensor_msgs/image_encodings.hpp"

TEST(TestRosImageToQImage, TestConvertRGB8)
{
  sensor_msgs::msg::Image msg;
  msg.height = 480;
  msg.width = 640;
  msg.encoding = sensor_msgs::image_encodings::RGB8;
  msg.is_bigendian = false;
  msg.step = 640 * 3;
  msg.data.resize(640 * 480 * 3, 0);

  auto qImage = ros_image_to_qimage::Convert(msg);
  ASSERT_EQ(qImage.width(), 640);
  ASSERT_EQ(qImage.height(), 480);
  ASSERT_EQ(qImage.format(), QImage::Format_RGB888);
}

TEST(TestRosImageToQImage, TestOptions)
{
  sensor_msgs::msg::Image msg;
  msg.height = 480;
  msg.width = 640;
  msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  msg.is_bigendian = false;
  msg.step = 640 * 4;
  msg.data.resize(640 * 480 * 4, 0);

  for (unsigned x = 0; x < 480; ++x) {
    for (unsigned y = 0; y < 640; ++y) {
      msg.data[(x * 640 + y) * 4 + 0] = x + y;
    }
  }

  cv_bridge::CvtColorForDisplayOptions options;
  options.do_dynamic_scaling = true;
  auto qImage = ros_image_to_qimage::Convert(msg, options);

  ASSERT_EQ(qImage.width(), 640);
  ASSERT_EQ(qImage.height(), 480);
  ASSERT_EQ(qImage.format(), QImage::Format_RGB888);
}
