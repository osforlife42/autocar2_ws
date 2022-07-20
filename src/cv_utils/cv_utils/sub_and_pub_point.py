from typing import Tuple
import numpy as np
import cv2
import rclpy  # Python library for ROS 2
from rclpy import qos
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image, CameraInfo  # Image is the message type
from geometry_msgs.msg import PointStamped, Point 
from visualization_msgs.msg import Marker 
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from cv_utils import math_utils

points_to_circle = []


# these info is taken from the /camera_info topic ! 
# NUM_PIXELS_X = 640
# NUM_PIXELS_Y = 480
# H_FOV = 1.5  # IN RADIANS (got from the camera model in lib.xacro)
# # IN RADIANS (got from the camera model in lib.xacro)
# V_FOV = 1.5*(NUM_PIXELS_Y/NUM_PIXELS_X)

# experimental parameter to calibrate the frame of refernce of the camera.
# check math_utils for more info
FRONT_CAMERA_THETA_SHIFT = -np.pi/2


class ImageSubscriber(Node):

    def __init__(self):
        """
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
        use_sim_time_param = rclpy.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([use_sim_time_param])

        cam = "front"
        self.cam_frame = f"camera_{cam}"
        image_topic_name = f"/camera_{cam}_sensor/image_raw"
        cam_info_topic_name = f"/camera_{cam}_sensor/camera_info"
        self.image_sub = self.create_subscription(
            Image,
            image_topic_name,
            self.listener_callback,
            qos_profile=qos.qos_profile_sensor_data)

        self._cam_info : CameraInfo = None
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            cam_info_topic_name,
            self.cam_info_cb,
            qos_profile=qos.qos_profile_sensor_data)
        self.image_sub # prevent unused variable warning
        self.cam_info_sub # prevent unused variable warning
        self._recieved_first_info = False

        self.position_publisher = self.create_publisher(
            PointStamped, '/clicked_point', qos_profile=2)
        self.line_publisher = self.create_publisher(
            Marker, '/marker', qos_profile=2)

        self.br = CvBridge()

        self.win = cv2.namedWindow("frame")
        cv2.setMouseCallback('frame', self.click_event)

    def click_event(self, event, x, y, flags, params):
        # checking for left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:

            # displaying the coordinates
            # on the Shell
            print(f"clicked on: {x}, {y}")

            global points_to_circle
            points_to_circle.append((x, y))
            # theta, phi = 0,np.pi/2
            # point_to_publish = math_utils.theta_phi_to_sphere_point(theta, phi)
            if not self._cam_info: 
                print(f"waiting for camera info to be available!")
                return 
            point_to_publish = math_utils.image_point_to_sphere_point(
                x, y, self.image_shape[1], self.image_shape[0], self.fov_x, self.fov_y, theta_shift=FRONT_CAMERA_THETA_SHIFT, phi_shift=0)
            self.publish_position(point_to_publish)
            self.publish_line_to_point(point_to_publish)

    def publish_line_to_point(self, line_point: Tuple[float, float, float], line_size: float = 0.01): 
        line = Marker()
        line.header.stamp = self.get_clock().now().to_msg()
        line.header.frame_id = self.cam_frame
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.scale.x = line_size
        line.scale.y = line_size 
        line.scale.z = line_size 
        line.color.g = 1.0
        line.color.a = 1.0

        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.0
        line.points.append(origin)
        end_point = Point()
        end_point.x = float(line_point[0])
        end_point.y = float(line_point[1])
        end_point.z = float(line_point[2])
        line.points.append(end_point)
        self.line_publisher.publish(line)

    def publish_position(self, position: Tuple[float, float, float]):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cam_frame
        msg.point.x = float(position[0])
        msg.point.y = float(position[1])
        msg.point.z = float(position[2])
        self.position_publisher.publish(msg)

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        for point in points_to_circle:
            cv2.circle(current_frame, point, 4, (0, 255, 0), -1)

        # Display image
        cv2.imshow("frame", current_frame)

        cv2.waitKey(1)

    def cam_info_cb(self, cam_info: CameraInfo): 
        self._cam_info = cam_info
        if not self._recieved_first_info: 
            print(f"recieved first cam info message - fov_x: {self.fov_x}, fov_y: {self.fov_y}, width: {self.image_shape[1]}, height: {self.image_shape[0]}")
        self._recieved_first_info = True

    @property
    def image_shape(self,): 
        """
        return (height, height) of the image from /camera_info topic
        """
        return (self._cam_info.height, self._cam_info.width) if self._cam_info else None

    @property
    def f_x(self,): 
        return self._cam_info.k[0] if self._cam_info else None

    @property
    def f_y(self,): 
        return self._cam_info.k[4] if self._cam_info else None

    @property
    def fov_x(self,): 
        if not self._cam_info: 
            return None 
        return 2 * np.arctan2(self.image_shape[1], 2*self.f_x)

    @property
    def fov_y(self,): 
        if not self._cam_info: 
            return None 
        return 2 * np.arctan2(self.image_shape[0], 2*self.f_y)

def main(args=None):

    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    # cleanup
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
