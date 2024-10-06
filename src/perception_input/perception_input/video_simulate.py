import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Header

class VideoSimulate(Node):
    def __init__(self):
        super().__init__('video_simulate')
        self.publisher = self.create_publisher(
            Image,
            '/zed/image_rect',
            10
        )
        self.bridge = CvBridge()
        self.video_capture = cv2.VideoCapture("/driverless-learning/output.avi")

    def publish(self):
        ret, frame = self.video_capture.read()
        image_msg = self.bridge.cv2_to_imgmsg(frame, 'passthrough')
        self.publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    video_simulate = VideoSimulate()
    try:
        while rclpy.ok():
            video_simulate.publish()
    except KeyboardInterrupt:
        pass
    finally:
        video_simulate.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()