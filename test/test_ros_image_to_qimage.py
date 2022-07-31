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

from ros_image_to_qimage import ros_image_to_qimage
from sensor_msgs.msg import Image


def test_convert():

    msg = Image(
        height=480,
        width=640,
        encoding='rgb8',
        is_bigendian=0,
        step=640*3,
        data=[0]*640*480*3,
    )
    q_image = ros_image_to_qimage.convert(msg)
    assert q_image is not None
    assert q_image.width() == 640
    assert q_image.height() == 480
