# Copyright 2022 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from cv_bridge import CvBridge, CvBridgeError

from python_qt_binding.QtGui import QImage

from sensor_msgs.msg import Image

cv_bridge = CvBridge()


def convert(msg: Image) -> QImage:

    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
        height, width, _ = cv_image.shape
        bytesPerLine = 3 * width
        return QImage(cv_image.data, width, height, bytesPerLine,
                      QImage.Format_RGB888)
    except CvBridgeError:
        return None
