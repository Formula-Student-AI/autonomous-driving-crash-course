"""OpenCV viewer for the EUFS emulated camera.

IMPORTANT: eufs_sim2 has NO sensor_msgs/Image camera topic. The ZED in
eufs_racecar/sensors/urdf/zed.urdf.xacro is TF frames + mesh only.
The 'camera' plugin (config/plugin_params.yaml camera: fov 1.91889 rad,
max range 20 m) publishes detections on /camera/cones
(eufs_msgs/ConeWithColorProbabilityArray, car frame, +x forward).

This node renders those detections as a synthetic forward camera view
with OpenCV (pinhole projection) + a bird-eye strip, via cv2.imshow.
If $DISPLAY is unset (headless), frames are written to /tmp/cam_view.png.

Also included: ImageViewer class (commented main) for a REAL image topic
via cv_bridge, if you later add a Gazebo/Isaac rendered camera.
"""
import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from eufs_msgs.msg import ConeWithColorProbabilityArray

# Horizontal FOV of emulated camera (must match plugin_params.yaml)
FOV_RAD = 1.91889
MAX_RANGE = 20.0
W, H = 640, 480


def cone_color(msg) -> tuple:
    """Pick BGR by highest probability."""
    probs = {'b': msg.blue_prob, 'y': msg.yellow_prob, 'o': msg.orange_prob}
    k = max(probs, key=probs.get)
    return {'b': (255, 0, 0), 'y': (0, 255, 255), 'o': (0, 165, 255)}[k]


class SyntheticCameraViewer(Node):
    def __init__(self):
        super().__init__('camera_opencv_viewer')
        self.sub = self.create_subscription(
            ConeWithColorProbabilityArray, '/camera/cones', self.cb, 10)
        self.fx = (W / 2.0) / math.tan(FOV_RAD / 2.0)
        self.get_logger().info('Subscribed to /camera/cones. Press q in window to quit.')
        self.headless = not os.environ.get('DISPLAY')
        if self.headless:
            self.get_logger().warn('No $DISPLAY; writing frames to /tmp/cam_view.png')

    def cb(self, msg: ConeWithColorProbabilityArray):
        img = np.zeros((H, W, 3), dtype=np.uint8)
        img[:] = (30, 30, 30)
        cv2.line(img, (0, H // 2), (W, H // 2), (60, 60, 60), 1)  # horizon
        for c in msg.cones:
            x, y = c.point.x, c.point.y  # car frame: x fwd, y left
            if x < 1.0 or x > MAX_RANGE:
                continue
            angle = math.atan2(y, x)
            if abs(angle) > FOV_RAD / 2:
                continue
            u = int(W / 2 - self.fx * math.tan(angle))  # left = smaller u
            scale = min(1.0, 8.0 / max(x, 0.5))
            v = int(H * 0.55 + (H * 0.4) * scale)
            r = max(4, int(18 * scale))
            cv2.circle(img, (u, v), r, cone_color(c), -1)
            cv2.circle(img, (u, v), r, (255, 255, 255), 1)
            cv2.putText(img, f'{x:.1f}m', (u - 20, v + r + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.putText(img, f'/camera/cones n={len(msg.cones)}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if self.headless:
            cv2.imwrite('/tmp/cam_view.png', img)
        else:
            cv2.imshow('emulated camera (/camera/cones)', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise SystemExit


# --- Template for a REAL image topic (needs cv_bridge + a rendered camera) ---
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# class ImageViewer(Node):
#     def __init__(self):
#         super().__init__('image_viewer')
#         self.bridge = CvBridge()
#         self.create_subscription(Image, '/zed/left/image_raw', self.cb, 10)
#     def cb(self, msg):
#         img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         cv2.imshow('camera', img); cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticCameraViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
