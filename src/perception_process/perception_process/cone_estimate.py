import rclpy
from rclpy.node import Node

from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, Point2, Color, Color
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance

from cv_bridge import CvBridge
from ultralytics import YOLO
from . import sort
import numpy as np

class ConeDetect(Node):
    def __init__(self):
        super().__init__('cone_detect')
        self.subscription = self.create_subscription(
            Image,
            '/zed/image_rect',
            self.cone_callback,
            10)
        self.subscription
        self.publisher_marker = self.create_publisher(
            ImageAnnotations,
            '/cones/image_annotations',
            10
        )
        self.publisher_depth = self.create_publisher(
            ConeArrayWithCovariance,
            '/cones',
            10
        )
        self.bridge = CvBridge()

        self.model = YOLO('/driverless-learning/src/perception_process/perception_process/best.pt')

    def cone_callback(self, msg):
        annotations, results = self.inference(msg)
        self.publisher_marker.publish(annotations)

        depths = self.depth_estimate(results)
        self.publish(depths)


    def inference(self, msg):
        classes = {
        "0": "yellow_cone",
        "1": "blue_cone",
        "2": "orange_cone",
        "3": "big_orange_cone",
        "4": "unknown_cone"
        }
        frame = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        results = self.model(frame[:, :frame.shape[1]//2])
        annotations = ImageAnnotations()
        processed_results = []

        for i in range(len(results[0].boxes.cls)):
            x, y, w, h = [float(x) for x in results[0].boxes.xywh[i].tolist()]
            annotation = PointsAnnotation(
                timestamp = self.get_clock().now().to_msg(),
                type = PointsAnnotation.LINE_LOOP,
                points = [
                    Point2(x=x+(w//2), y=y+(h//2)),
                    Point2(x=x+(w//2), y=y-(h//2)),
                    Point2(x=x-(w//2), y=y-(h//2)),
                    Point2(x=x-(w//2), y=y+(h//2))
                ],
                outline_color = Color(r=0.0, g=1.0, b=0.0, a=1.0),
                thickness = 5.0
            )
            annotations.points.append(annotation)

            processed_results.append({
                'confidence': results[0].boxes.conf[i].item(),
                'cone_type': classes[str(int(results[0].boxes.cls[i].item()))],
                'x': x,
                'y': y,
                'w': w,
                'h': h,
                'image': msg
            })
        return annotations, processed_results
    
    def depth_estimate(self, results):
        depths = []

        for cone_type in ["blue_cone", "yellow_cone", "orange_cone", "big_orange_cone", "unknown_color_node"]:
            filtered_results = [x for x in results if x["cone_type"] == cone_type and x["confidence"] >= 0.8]
            sorted_results = sort.sort_results(filtered_results)
            if len(sorted_results) > 0:
                half_frame_width = sorted_results[0]['image'].width//2
                previous_disparity = half_frame_width

            for result in sorted_results:
                xl = int(result["x"])
                y = int(result["y"])
                width = int(result["w"])
                height = int(result["h"])

                xr = self.get_right_point(self.bridge.imgmsg_to_cv2(result['image'], 'passthrough'), xl, y, width, height, previous_disparity)

                previous_disparity = xl - xr
                if previous_disparity == 0:
                    previous_disparity = 0.1

                depth = (0.11995 * self.get_focal_length(half_frame_width))/(previous_disparity)
                latitude = ((xl-(half_frame_width/2))*depth)/self.get_focal_length(half_frame_width)

                depths.append({
                    'cone_type': result['cone_type'],
                    'x': depth,
                    'y': latitude
                })

        print(depths)
        return depths
    
    def get_right_point(self, image, xl, y, width, height, previous_disparity):
        left_image = image[:, :image.shape[1]//2]
        right_image = image[:, image.shape[1]//2:]
        top = y - height//2
        bottom = y + height//2
        if top < 0:
            top = 0
        if bottom >= len(left_image):
            bottom = len(left_image) - 1
        left = xl - width//2
        right = xl + width//2
        if left < 0:
            left = 0
        if right >= len(left_image[0]):
            right = len(left_image[0]) - 1

        template = left_image[top:bottom, left:right]


        min_similarity = float('inf')
        xr = xl

        start = int(xl - previous_disparity)
        xl = int(xl)

        if start < width//2:
            start = width//2

        for xi in range(start, xl):
            window = right_image[top:bottom, xi - width//2 : xi + width//2]
            similarity = np.sum((template - window)**2)
            if similarity < min_similarity:
                min_similarity = similarity
                xr = xi
        return xr
    
    def get_focal_length(self, frame_width):
        if frame_width == 2208 :
            return 1067.11
        if frame_width == 1920 :
            return 1068.1
        if frame_width == 1280 :
            return 534.05
        if frame_width == 672 :
            return 267.025

    def publish(self, depths):
        cone_array = ConeArrayWithCovariance()
        for depth in depths:
            cone = ConeWithCovariance()
            cone.point.x = depth['x']
            cone.point.y = -1*depth['y']
            cone.covariance = np.zeros((2, 2)).flatten().tolist()
            
            if depth['cone_type'] == 'yellow_cone':
                cone_array.yellow_cones.append(cone)
            elif depth['cone_type'] == 'blue_cone':
                cone_array.blue_cones.append(cone)
            elif depth['cone_type'] == 'orange_cone':
                cone_array.orange_cones.append(cone)
            elif depth['cone_type'] == 'big_orange_cone':
                cone_array.big_orange_cones.append(cone)
            elif depth['cone_type'] == 'unknown_cone':
                cone_array.unknown_cones.append(cone)
        
        self.publisher_depth.publish(cone_array)
    
def main(args=None):
    rclpy.init(args=args)
    cone_detect = ConeDetect()
    rclpy.spin(cone_detect)
    cone_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()