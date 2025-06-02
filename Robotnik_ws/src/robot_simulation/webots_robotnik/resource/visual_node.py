import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class HarukoFollower(Node):
    def __init__(self):
        super().__init__('haruko_follower')

        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscripciones a sensores
        self.create_subscription(Image, '/camera_front/image_raw', self.image_callback, 10)
        self.create_subscription(PointCloud2, '/camera_front/depth/points', self.pointcloud_callback, 10)

        self.bridge = CvBridge()
        self.latest_cloud = None
        self.haruko_detected = False
        self.haruko_position_x = None
        self.image_width = None

        self.state = 'BUSCANDO_ARUCO'
        self.state_start_time = time.time()
        self.timer = self.create_timer(0.1, self.control_loop)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.image_width is None:
            self.image_width = gray.shape[1]

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            self.haruko_detected = True
            # Calcular posición media del marcador
            (cX, _) = np.mean(corners[0][0], axis=0)
            self.haruko_position_x = cX
        else:
            self.haruko_detected = False
            self.haruko_position_x = None

    def pointcloud_callback(self, msg):
        self.latest_cloud = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    def control_loop(self):
        twist = Twist()
        current_time = time.time()

        if self.state == 'BUSCANDO_ARUCO':
            print("[Estado] BUSCANDO_ARUCO")
            if self.haruko_detected:
                self.state = 'ACERCANDOSE_ARUCO'
                self.state_start_time = current_time
                print("--> Haruko detectado, cambiando a ACERCANDOSE_ARUCO")
            elif current_time - self.state_start_time > 10:
                self.state = 'MODO_EVASION'
                self.state_start_time = current_time
                print("--> Tiempo agotado, cambiando a MODO_EVASION")
            else:
                twist.angular.z = 0.3

        elif self.state == 'ACERCANDOSE_ARUCO':
            print("[Estado] ACERCANDOSE_ARUCO")
            if self.haruko_position_x is not None and self.image_width is not None:
                error_x = self.haruko_position_x - (self.image_width / 2)
                threshold = 30

                if abs(error_x) > threshold:
                    twist.angular.z = -0.002 * error_x
                    print(f"--> Ajustando orientación: error_x = {error_x}")
                else:
                    twist.linear.x = 0.2
                    print("--> Avanzando hacia el Haruko")

        elif self.state == 'MODO_EVASION':
            print("[Estado] MODO_EVASION")
            if current_time - self.state_start_time > 10:
                self.state = 'BUSCANDO_ARUCO'
                self.state_start_time = current_time
                print("--> Volviendo a BUSCANDO_ARUCO")
            elif self.latest_cloud:
                front_points = [p for p in self.latest_cloud if -0.2 < p[1] < 0.2 and 0 < p[0] < 1.0]
                if len(front_points) > 50:
                    twist.angular.z = 0.5
                    print("--> Objeto cerca al frente, girando")
                else:
                    twist.linear.x = 0.2
                    print("--> Avanzando evitando obstáculos")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = HarukoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

