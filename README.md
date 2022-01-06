# ros_image_to_qimage

[![Build and Test (foxy)](../../actions/workflows/build_and_test_foxy.yaml/badge.svg)](../../actions/workflows/build_and_test_foxy.yaml)
[![Build and Test (galactic)](../../actions/workflows/build_and_test_galactic.yaml/badge.svg)](../../actions/workflows/build_and_test_galactic.yaml)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg)](../../actions/workflows/build_and_test_rolling.yaml)

Converts a ROS2 `sensor_msgs/msg/Image` to QImage.

# Usage

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

