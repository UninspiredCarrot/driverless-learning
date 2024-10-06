import configparser
import cv2

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

class ZedCameraRectify(Node):
    def __init__(self):
        super().__init__('zed_rectify')
        self.publisher_raw = self.create_publisher(
            Image, 
            '/zed/image_raw', 
            10)
        self.publisher_rect = self.create_publisher(
            Image, 
            '/zed/image_rect', 
            10)
        self.bridge = CvBridge()
        self.video_capture = cv2.VideoCapture(0)
        self.ebs_srv = self.create_client(Trigger, "/ros_can/ebs")

    def publish(self):
        ret, raw_frame = self.video_capture.read()

        if not ret:
            self.get_logger().error('OpenCV not able to read image')
            self.requestEBS()

        raw_image_msg = self.bridge.cv2_to_imgmsg(raw_frame, 'passthrough')

        self.publisher_raw.publish(raw_image_msg)

        rect_frame = self.rectify(raw_frame)
        rect_image_msg = self.bridge.cv2_to_imgmsg(rect_frame, 'passthrough')
        self.publisher_rect.publish(rect_image_msg)

    def requestEBS(self):
            self.get_logger().debug("Requesting EBS")
            if self.ebs_srv.wait_for_service(timeout_sec=1):
                request = Trigger.Request()
                result = self.ebs_srv.call_async(request)
                self.get_logger().debug("EBS successful")
                self.get_logger().debug(result)
            else:
                self.get_logger().warn(
                    "/ros_can/ebs service is not available")


    def rectify(self, raw_image):
        height, width, _ = raw_image.shape
        camera_matrix_left, camera_matrix_right, map_left_x, map_left_y, map_right_x, map_right_y = self.calibrate(height, width)
        left_frame = raw_image[:, :width//2]
        right_frame = raw_image[:, width//2:]
        left_rect = cv2.remap(left_frame, map_left_x, map_left_y, interpolation=cv2.INTER_LINEAR)[:, :width//2]
        right_rect = cv2.remap(right_frame, map_right_x, map_right_y, interpolation=cv2.INTER_LINEAR)[:, :width//2]

        rect_frame = np.concatenate((left_rect, right_rect), axis=1)
        return rect_frame
    
    def calibrate(self, height, width):
        cameraMarix_left = cameraMatrix_right = map_left_y = map_left_x = map_right_y = map_right_x = np.array([])

        config = configparser.ConfigParser()
        config.read("/driverless-learning/src/perception_input/perception_input/SN34281902.conf")
        if width == 2208*2 :
            resolution_str = '2K'
        elif width == 1920*2 :
            resolution_str = 'FHD'
        elif width == 1280*2 :
            resolution_str = 'HD'
        elif width == 672*2 :
            resolution_str = 'VGA'
        else:
            resolution_str = 'HD'

        T_ = np.array([-float(config['STEREO']['Baseline'] if 'Baseline' in config['STEREO'] else 0),
                    float(config['STEREO']['TY_'+resolution_str] if 'TY_'+resolution_str in config['STEREO'] else 0),
                    float(config['STEREO']['TZ_'+resolution_str] if 'TZ_'+resolution_str in config['STEREO'] else 0)])

        left_cam_cx = float(config['LEFT_CAM_'+resolution_str]['cx'] if 'cx' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_cy = float(config['LEFT_CAM_'+resolution_str]['cy'] if 'cy' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_fx = float(config['LEFT_CAM_'+resolution_str]['fx'] if 'fx' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_fy = float(config['LEFT_CAM_'+resolution_str]['fy'] if 'fy' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_k1 = float(config['LEFT_CAM_'+resolution_str]['k1'] if 'k1' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_k2 = float(config['LEFT_CAM_'+resolution_str]['k2'] if 'k2' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_p1 = float(config['LEFT_CAM_'+resolution_str]['p1'] if 'p1' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_p2 = float(config['LEFT_CAM_'+resolution_str]['p2'] if 'p2' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_p3 = float(config['LEFT_CAM_'+resolution_str]['p3'] if 'p3' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_k3 = float(config['LEFT_CAM_'+resolution_str]['k3'] if 'k3' in config['LEFT_CAM_'+resolution_str] else 0)

        right_cam_cx = float(config['RIGHT_CAM_'+resolution_str]['cx'] if 'cx' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_cy = float(config['RIGHT_CAM_'+resolution_str]['cy'] if 'cy' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_fx = float(config['RIGHT_CAM_'+resolution_str]['fx'] if 'fx' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_fy = float(config['RIGHT_CAM_'+resolution_str]['fy'] if 'fy' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_k1 = float(config['RIGHT_CAM_'+resolution_str]['k1'] if 'k1' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_k2 = float(config['RIGHT_CAM_'+resolution_str]['k2'] if 'k2' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_p1 = float(config['RIGHT_CAM_'+resolution_str]['p1'] if 'p1' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_p2 = float(config['RIGHT_CAM_'+resolution_str]['p2'] if 'p2' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_p3 = float(config['RIGHT_CAM_'+resolution_str]['p3'] if 'p3' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_k3 = float(config['RIGHT_CAM_'+resolution_str]['k3'] if 'k3' in config['RIGHT_CAM_'+resolution_str] else 0)

        R_zed = np.array([float(config['STEREO']['RX_'+resolution_str] if 'RX_' + resolution_str in config['STEREO'] else 0),
                        float(config['STEREO']['CV_'+resolution_str] if 'CV_' + resolution_str in config['STEREO'] else 0),
                        float(config['STEREO']['RZ_'+resolution_str] if 'RZ_' + resolution_str in config['STEREO'] else 0)])

        R, _ = cv2.Rodrigues(R_zed)
        cameraMatrix_left = np.array([[left_cam_fx, 0, left_cam_cx],
                            [0, left_cam_fy, left_cam_cy],
                            [0, 0, 1]])

        cameraMatrix_right = np.array([[right_cam_fx, 0, right_cam_cx],
                            [0, right_cam_fy, right_cam_cy],
                            [0, 0, 1]])

        distCoeffs_left = np.array([[left_cam_k1], [left_cam_k2], [left_cam_p1], [left_cam_p2], [left_cam_k3]])

        distCoeffs_right = np.array([[right_cam_k1], [right_cam_k2], [right_cam_p1], [right_cam_p2], [right_cam_k3]])

        T = np.array([[T_[0]], [T_[1]], [T_[2]]])
        R1 = R2 = P1 = P2 = np.array([])

        R1, R2, P1, P2 = cv2.stereoRectify(cameraMatrix1=cameraMatrix_left,
                                        cameraMatrix2=cameraMatrix_right,
                                        distCoeffs1=distCoeffs_left,
                                        distCoeffs2=distCoeffs_right,
                                        R=R, T=T,
                                        flags=cv2.CALIB_ZERO_DISPARITY,
                                        alpha=0,
                                        imageSize=(width, height),
                                        newImageSize=(width, height))[0:4]

        map_left_x, map_left_y = cv2.initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, R1, P1, (width, height), cv2.CV_32FC1)
        map_right_x, map_right_y = cv2.initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, (width, height), cv2.CV_32FC1)

        cameraMatrix_left = P1
        cameraMatrix_right = P2

        return cameraMatrix_left, cameraMatrix_right, map_left_x, map_left_y, map_right_x, map_right_y

def main(args=None):
    rclpy.init(args=args)
    zed_camera_rectify = ZedCameraRectify()
    try:
        while rclpy.ok():
            zed_camera_rectify.publish()
    except KeyboardInterrupt:
        pass
    finally:
        zed_camera_rectify.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()