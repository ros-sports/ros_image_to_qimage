# ros_image_to_qimage

[![Build and Test (foxy)](https://github.com/ros-sports/ros_image_to_qimage/actions/workflows/build_and_test_foxy.yaml/badge.svg?branch=foxy)](https://github.com/ros-sports/ros_image_to_qimage/actions/workflows/build_and_test_foxy.yaml?query=branch:foxy)
[![Build and Test (galactic)](https://github.com/ros-sports/ros_image_to_qimage/actions/workflows/build_and_test_galactic.yaml/badge.svg?branch=galactic)](https://github.com/ros-sports/ros_image_to_qimage/actions/workflows/build_and_test_galactic.yaml?query=branch:galactic)
[![Build and Test (humble)](https://github.com/ros-sports/ros_image_to_qimage/actions/workflows/build_and_test_humble.yaml/badge.svg?branch=humble)](https://github.com/ros-sports/ros_image_to_qimage/actions/workflows/build_and_test_humble.yaml?query=branch:humble)
[![Build and Test (rolling)](https://github.com/ros-sports/ros_image_to_qimage/actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](https://github.com/ros-sports/ros_image_to_qimage/actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

Converts a ROS2 `sensor_msgs/msg/Image` to QImage.

# Installation

### ROS2 Galactic / Humble / Rolling

Binary installation is available. Source your ROS installation, then run:

```
sudo apt install ros-${ROS_DISTRO}-ros-image-to-qimage
```

Alternatively to build from source, source your ROS installation, then run the following in your ROS workspace:

```
git clone https://github.com/ros-sports/ros_image_to_qimage.git src/ros_image_to_qimage --branch ${ROS_DISTRO}
colcon build
```

### ROS2 Foxy

Only source installation is available. Source your ROS installation, then run the following in your ROS workspace:

```
git clone https://github.com/ros-sports/ros_image_to_qimage.git src/ros_image_to_qimage --branch ${ROS_DISTRO}
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
