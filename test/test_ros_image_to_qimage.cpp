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

// TEST(TestRosImageToQImage, TestOptions)
// {
//   sensor_msgs::msg::Image msg;
//   int height = 2;
//   int width = 3;
//   msg.height = height;
//   msg.width = width;
//   msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
//   msg.is_bigendian = false;
//   msg.step = width * 4;
//   msg.data.resize(width * height * 4, 0);
//   msg.data = []

//   for (unsigned x = 0; x < height; ++x) {
//     for (unsigned y = 0; y < width; ++y) {
//       msg.data[(x * width + y) * 4 + 0] = x + y;
//     }
//   }

//   cv_bridge::CvtColorForDisplayOptions options;
//   options.min_image_value = 0.0;
//   options.max_image_value = 10.0;
//   auto qImage = ros_image_to_qimage::Convert(msg, options);

//   ASSERT_EQ(qImage.width(), width);
//   ASSERT_EQ(qImage.height(), height);
//   ASSERT_EQ(qImage.format(), QImage::Format_RGB888);
// }


TEST(TestRosImageToQimage, TestSimple)
{
  std::vector<std::string> encodings = {
    sensor_msgs::image_encodings::RGB8,
    sensor_msgs::image_encodings::RGBA8,
    sensor_msgs::image_encodings::RGB16,
    sensor_msgs::image_encodings::RGBA16,
    sensor_msgs::image_encodings::BGR8,
    sensor_msgs::image_encodings::BGRA8,
    sensor_msgs::image_encodings::BGR16,
    sensor_msgs::image_encodings::BGRA16,
    sensor_msgs::image_encodings::MONO8,
    sensor_msgs::image_encodings::MONO16,
  };

  for (const auto & encoding : encodings)
  {
    int height = 2;
    int width = 3;
    int channels = sensor_msgs::image_encodings::numChannels(encoding);
    int bitDepth = sensor_msgs::image_encodings::bitDepth(encoding);

    sensor_msgs::msg::Image msg;
    msg.height = height;
    msg.width = width;
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = msg.width * channels * bitDepth;
    msg.data.resize(width * height * channels * bitDepth, 0);

    QImage qImage;
    EXPECT_NO_THROW(qImage = ros_image_to_qimage::Convert(msg));
    EXPECT_EQ(qImage.width(), width);
    EXPECT_EQ(qImage.height(), height);
    EXPECT_EQ(qImage.format(), QImage::Format_RGB888);
  }
}


TEST(TestRosImageToQimage, TestCvTypes)
{
  std::vector<std::string> encodings = {
    sensor_msgs::image_encodings::TYPE_8UC1,
    sensor_msgs::image_encodings::TYPE_8UC2,
    sensor_msgs::image_encodings::TYPE_8UC3,
    sensor_msgs::image_encodings::TYPE_8UC4,
    sensor_msgs::image_encodings::TYPE_8SC1,
    sensor_msgs::image_encodings::TYPE_8SC2,
    sensor_msgs::image_encodings::TYPE_8SC3,
    sensor_msgs::image_encodings::TYPE_8SC4,
    sensor_msgs::image_encodings::TYPE_16UC1,
    sensor_msgs::image_encodings::TYPE_16UC2,
    sensor_msgs::image_encodings::TYPE_16UC3,
    sensor_msgs::image_encodings::TYPE_16UC4,
    sensor_msgs::image_encodings::TYPE_16SC1,
    sensor_msgs::image_encodings::TYPE_16SC2,
    sensor_msgs::image_encodings::TYPE_16SC3,
    sensor_msgs::image_encodings::TYPE_16SC4,
    sensor_msgs::image_encodings::TYPE_32SC1,
    sensor_msgs::image_encodings::TYPE_32SC2,
    sensor_msgs::image_encodings::TYPE_32SC3,
    sensor_msgs::image_encodings::TYPE_32SC4,
    sensor_msgs::image_encodings::TYPE_32FC1,
    sensor_msgs::image_encodings::TYPE_32FC2,
    sensor_msgs::image_encodings::TYPE_32FC3,
    sensor_msgs::image_encodings::TYPE_32FC4,
    sensor_msgs::image_encodings::TYPE_64FC1,
    sensor_msgs::image_encodings::TYPE_64FC2,
    sensor_msgs::image_encodings::TYPE_64FC3,
    sensor_msgs::image_encodings::TYPE_64FC4,
  };

  for (const auto & encoding : encodings)
  {
    int height = 2;
    int width = 3;
    int channels = sensor_msgs::image_encodings::numChannels(encoding);
    int bitDepth = sensor_msgs::image_encodings::bitDepth(encoding);

    sensor_msgs::msg::Image msg;
    msg.height = height;
    msg.width = width;
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = msg.width * channels * bitDepth;
    msg.data.resize(width * height * channels * bitDepth, 0);

    QImage qImage;
    EXPECT_NO_THROW(qImage = ros_image_to_qimage::Convert(msg));
    EXPECT_EQ(qImage.width(), width);
    EXPECT_EQ(qImage.height(), height);
    EXPECT_EQ(qImage.format(), QImage::Format_RGB888);
  }
}

