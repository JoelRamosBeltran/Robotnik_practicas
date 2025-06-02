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
        self.cmd_pub = self.create_publisher(Twist, '/rbwatcher/diffdrive_controller/cmd_vel_unstamped', 10)

        # Subscripciones a sensores
        self.create_subscription(Image, '/rbwatcher/rbwatcher/rbwatcher/front_camera_color/image_color', self.image_callback, 10)
        self.create_subscription(PointCloud2, '/rbwatcher/rbwatcher/rbwatcher/front_camera_depth/point_cloud', self.pointcloud_callback, 10)

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

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 7  # Reduce la constante de umbral adaptativo (menos contraste necesario)
        parameters.minMarkerPerimeterRate = 0.02  # Detecta marcadores más pequeños
        parameters.maxMarkerPerimeterRate = 4.0 # Amplía el rango de tamaño permitido
        parameters.polygonalApproxAccuracyRate = 0.05  # Permite detectar esquinas menos perfectas
        parameters.minCornerDistanceRate = 0.05  # Permite marcadores más cerca unos de otros
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # Mejora la precisión de las esquinas                
        parameters.minDistanceToBorder = 3  # Permite marcadores más cerca unos de otros
        parameters.errorCorrectionRate = 0.3 # Mayor tolerancia a errores en el marcador

        
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            self.haruko_detected = True
            (cX, _) = np.mean(corners[0][0], axis=0)
            self.haruko_position_x = cX
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            self.haruko_detected = False
            self.haruko_position_x = None
        
        cv2.putText(
	    frame,
	    f"Estado: {self.state}",  # texto a mostrar
	    (10, 30),  # posición (x, y) en píxeles
	    cv2.FONT_HERSHEY_SIMPLEX,  # tipo de fuente
	    1,  # tamaño de fuente
	    (0, 255, 0),  # color (verde)
	    2,  # grosor del texto
	    cv2.LINE_AA  # suavizado
	)
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)
        

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
            elif current_time - self.state_start_time > 50:
                self.state = 'MODO_EVASION'
                self.state_start_time = current_time
                print("--> Tiempo agotado, cambiando a MODO_EVASION")
            else:
                twist.angular.z = 0.1

        elif self.state == 'ACERCANDOSE_ARUCO':
            print("[Estado] ACERCANDOSE_ARUCO")
            if self.haruko_position_x is not None and self.image_width is not None:
                error_x = self.haruko_position_x - (self.image_width / 2)
                threshold = 30

                if abs(error_x) > threshold:
                    twist.angular.z = -0.001 * error_x
                    print(f"--> Ajustando orientación: error_x = {error_x}")
                else:
                    twist.linear.x = 0.4
                    print("--> Avanzando hacia el Haruko")
            else:
            	twist.angular.z = -0.1
            	self.state = 'BUSCANDO_ARUCO'

        elif self.state == 'MODO_EVASION':
            print("[Estado] MODO_EVASION")
            if current_time - self.state_start_time > 50:
                self.state = 'BUSCANDO_ARUCO'
                self.state_start_time = current_time
                print("--> Volviendo a BUSCANDO_ARUCO")
            
            elif self.haruko_detected:
                self.state = 'ACERCANDOSE_ARUCO'
                self.state_start_time = current_time
            
            elif self.latest_cloud:
                front_points = [p for p in self.latest_cloud if -0.2 < p[1] < 0.2 and 0 < p[0] < 1.0 and p[2] > 0.2]
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

