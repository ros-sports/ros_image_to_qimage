# ros_image_to_qimage

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=humble)](../../actions/workflows/build_and_test_humble.yaml?query=branch:humble)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_iron.yaml?query=branch:rolling)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

Converts a ROS2 `sensor_msgs/msg/Image` to QImage.

# Installation

### ROS2 Humble / Humble / Rolling

## Binary Installation

Binary installation is available. Source your ROS installation, then run:

```
sudo apt install ros-${ROS_DISTRO}-ros-image-to-qimage
```

## Source Installation

Alternatively to build from source, source your ROS installation, then run the following in your ROS workspace:

```
// For ROS 2 Iron / Rolling
git clone https://github.com/ros-sports/ros_image_to_qimage.git src/ros_image_to_qimage
colcon build

// For ROS 2 Humble
git clone https://github.com/ros-sports/ros_image_to_qimage.git src/ros_image_to_qimage --branch humble
colcon build
```

# Usage

## C++

```cpp
#include "ros_image_to_qimage/ros_image_to_qimage.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  QImage qImage = ros_image_to_qimage::Convert(*msg);
}
```

For your package.xml, add
```xml
<depend>ros_image_to_qimage</depend>
```

For your CMakeLists.txt, suppose we want to link `my_target` against this library:
```cmake
find_package(ros_image_to_qimage REQUIRED)
ament_target_dependencies(my_target ros_image_to_qimage)
```

## Python

```py
from ros_image_to_qimage import ros_image_to_qimage

def image_callback(self, msg):
    qimage = ros_image_to_qimage.convert(msg)
```

For your package.xml, add
```xml
<exec_depend>ros_image_to_qimage</exec_depend>
```
