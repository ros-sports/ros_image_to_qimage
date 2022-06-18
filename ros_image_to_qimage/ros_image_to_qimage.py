from sensor_msgs.msg import Image
from python_qt_binding.QtGui import QImage
from cv_bridge import CvBridge, CvBridgeError

cv_bridge = CvBridge()


def convert(msg: Image) -> QImage:

    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        height, width, _ = cv_image.shape
        bytesPerLine = 3 * width
        return QImage(cv_image.data, width, height, bytesPerLine,
                      QImage.Format_RGB888)
    except CvBridgeError:
        return None
