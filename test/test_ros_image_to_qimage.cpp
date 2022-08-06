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

std::vector<std::string> supportedEncodings = {
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
  sensor_msgs::image_encodings::TYPE_8UC1,
  sensor_msgs::image_encodings::TYPE_8UC3,
  sensor_msgs::image_encodings::TYPE_8UC4,
  sensor_msgs::image_encodings::TYPE_8SC1,
  sensor_msgs::image_encodings::TYPE_8SC3,
  sensor_msgs::image_encodings::TYPE_8SC4,
  sensor_msgs::image_encodings::TYPE_16UC1,
  sensor_msgs::image_encodings::TYPE_16UC3,
  sensor_msgs::image_encodings::TYPE_16UC4,
  sensor_msgs::image_encodings::TYPE_16SC1,
  sensor_msgs::image_encodings::TYPE_16SC3,
  sensor_msgs::image_encodings::TYPE_16SC4,
  sensor_msgs::image_encodings::TYPE_32SC1,
  sensor_msgs::image_encodings::TYPE_32SC3,
  sensor_msgs::image_encodings::TYPE_32SC4,
  sensor_msgs::image_encodings::TYPE_32FC1,
  sensor_msgs::image_encodings::TYPE_32FC3,
  sensor_msgs::image_encodings::TYPE_32FC4,
  sensor_msgs::image_encodings::TYPE_64FC1,
  sensor_msgs::image_encodings::TYPE_64FC3,
  sensor_msgs::image_encodings::TYPE_64FC4,
  sensor_msgs::image_encodings::BAYER_RGGB8,
  sensor_msgs::image_encodings::BAYER_BGGR8,
  sensor_msgs::image_encodings::BAYER_GBRG8,
  sensor_msgs::image_encodings::BAYER_GRBG8,
  sensor_msgs::image_encodings::BAYER_RGGB16,
  sensor_msgs::image_encodings::BAYER_BGGR16,
  sensor_msgs::image_encodings::BAYER_GBRG16,
  sensor_msgs::image_encodings::BAYER_GRBG16,
};

std::vector<std::string> unsupportedEncodings = {
  sensor_msgs::image_encodings::TYPE_8UC2,
  sensor_msgs::image_encodings::TYPE_8SC2,
  sensor_msgs::image_encodings::TYPE_16UC2,
  sensor_msgs::image_encodings::TYPE_16SC2,
  sensor_msgs::image_encodings::TYPE_32SC2,
  sensor_msgs::image_encodings::TYPE_32FC2,
  sensor_msgs::image_encodings::TYPE_64FC2,
  sensor_msgs::image_encodings::YUV422,
  sensor_msgs::image_encodings::YUV422_YUY2,
  sensor_msgs::image_encodings::NV21,
  sensor_msgs::image_encodings::NV24,
};

TEST(TestRosImageToQImage, TestConvertSupportedEncodings)
{
  for (const auto & encoding : supportedEncodings) {
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

    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = true;

    // std::cout << "encoding: " << encoding << std::endl;
    QImage qImage;
    EXPECT_NO_THROW(qImage = ros_image_to_qimage::Convert(msg, options)) << "Test failure with encoding: " << encoding;
    EXPECT_EQ(qImage.width(), width);
    EXPECT_EQ(qImage.height(), height);
    EXPECT_EQ(qImage.format(), QImage::Format_RGB888);
  }
}

TEST(TestRosImageToQImage, TestConvertUnsupportedEncodings)
{
  for (const auto & encoding : unsupportedEncodings) {
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

    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = true;

    EXPECT_THROW(auto qImage = ros_image_to_qimage::Convert(msg, options), std::runtime_error);
  }
}

TEST(TestRosImageToQImage, TestIsEncodingSupported_AllEncodings)
{
  for (const auto & encoding : supportedEncodings) {
    EXPECT_TRUE(ros_image_to_qimage::IsEncodingSupported(encoding));
  }

  for (const auto & encoding : unsupportedEncodings) {
    EXPECT_FALSE(ros_image_to_qimage::IsEncodingSupported(encoding));
  }

  // Random non-ROS encoding
  EXPECT_THROW(ros_image_to_qimage::IsEncodingSupported("bla"), std::runtime_error);
}
