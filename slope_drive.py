import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Float64MultiArray, Float32
import cv2
import cv2.ximgproc
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation
from slope_drive.utils.pruning import prune_branches


class UnifiedDriveNode(Node):
    def __init__(self):
        super().__init__('unified_drive_node')
        self.get_logger().info('Unified Drive Node has been started.')

        self.bridge = CvBridge()

        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                # BEV Parameters
                ('camera_height_m', 0.5),
                ('camera_pitch_offset_deg', 10.0),
                ('camera_roll_offset_deg', 0.0),
                ('bev_img_width', 400),
                ('bev_img_height', 400),
                ('bev_meters_per_pixel', 0.05),
                ('lane_threshold', 200),
                ('morph_kernel_size', 5),
                
                # Pruning Parameters
                ('length_threshold', 10),
                
                # Path Planner Parameters
                ('lookahead_distance_pixels', 50),
                ('steering_gain', 0.005),
                ('max_speed', 0.2)
            ])
        self.load_parameters()
        self.add_on_set_parameters_callback(self.parameters_callback)

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos_profile_sensor_data)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.cam_info_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(
            Imu, '/camera/camera/imu', self.imu_callback, qos_profile_sensor_data)

        # --- Publishers ---
        # Control Outputs
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(PointStamped, '/planning/target_point', 10)
        self.steering_debug_pub = self.create_publisher(Float32, '/planning/steering_debug', 10)
        
        # Intermediate Visual/Data Outputs (For visualizer_node and debugging)
        self.bev_image_pub = self.create_publisher(Image, '/bev/image', 10)
        self.mask_pub = self.create_publisher(Image, '/bev/mask', 10)
        self.skeleton_pub = self.create_publisher(Image, '/skeleton_image', 10)
        self.pruned_skeleton_pub = self.create_publisher(Image, '/pruned_skeleton', 10)
        self.debug_image_pub = self.create_publisher(Image, '/bev/debug_overlay', 10)
        self.homography_pub = self.create_publisher(Float64MultiArray, '/bev/H', 10)
        
        # Relay Outputs
        self.orientation_publisher = self.create_publisher(Imu, '/imu/orientation', 10)

        # --- State Variables ---
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_imu_msg = None
        self.image_to_ground_homography = None

    def load_parameters(self):
        self.camera_height = self.get_parameter('camera_height_m').value
        self.pitch_offset_rad = np.deg2rad(self.get_parameter('camera_pitch_offset_deg').value)
        self.roll_offset_rad = np.deg2rad(self.get_parameter('camera_roll_offset_deg').value)
        self.bev_width = self.get_parameter('bev_img_width').value
        self.bev_height = self.get_parameter('bev_img_height').value
        self.mpp = self.get_parameter('bev_meters_per_pixel').value
        self.lane_threshold = self.get_parameter('lane_threshold').value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        
        self.length_threshold = self.get_parameter('length_threshold').value
        self.lookahead_dist = self.get_parameter('lookahead_distance_pixels').value
        self.k_p = self.get_parameter('steering_gain').value
        self.max_speed = self.get_parameter('max_speed').value

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'camera_height_m':
                self.camera_height = param.value
            elif param.name == 'camera_pitch_offset_deg':
                self.pitch_offset_rad = np.deg2rad(param.value)
            elif param.name == 'camera_roll_offset_deg':
                self.roll_offset_rad = np.deg2rad(param.value)
            elif param.name == 'bev_img_width':
                self.bev_width = param.value
            elif param.name == 'bev_img_height':
                self.bev_height = param.value
            elif param.name == 'bev_meters_per_pixel':
                self.mpp = param.value
            elif param.name == 'lane_threshold':
                self.lane_threshold = param.value
            elif param.name == 'morph_kernel_size':
                self.morph_kernel_size = param.value
            elif param.name == 'length_threshold':
                self.length_threshold = param.value
            elif param.name == 'lookahead_distance_pixels':
                self.lookahead_dist = param.value
            elif param.name == 'steering_gain':
                self.k_p = param.value
            elif param.name == 'max_speed':
                self.max_speed = param.value
        
        if self.latest_imu_msg is not None:
            self.update_homography(self.latest_imu_msg)
        return SetParametersResult(successful=True)

    def cam_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.destroy_subscription(self.cam_info_sub)
            self.get_logger().info('Camera info received and subscription destroyed.')

    def imu_callback(self, msg):
        # IMU Orientation Node Relay 
        self.orientation_publisher.publish(msg)
        self.latest_imu_msg = msg
        if self.camera_matrix is not None:
            self.update_homography(msg)

    def update_homography(self, imu_msg):
        q = imu_msg.orientation
        R_world_to_imu = Rotation.from_quat([q.x, q.y, q.z, q.w])
        R_imu_to_cam_body = Rotation.from_euler('xyz', [self.roll_offset_rad, self.pitch_offset_rad, 0])
        R_world_to_cam_body = R_imu_to_cam_body * R_world_to_imu
        R_body_to_optical = Rotation.from_matrix([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
        R_world_to_cam_optical = R_body_to_optical * R_world_to_cam_body
        
        R = R_world_to_cam_optical.as_matrix().T
        t_cam_in_world = np.array([0, 0, self.camera_height])
        t_vec = -R @ t_cam_in_world
        
        H_g2i = self.camera_matrix @ np.hstack((R[:, 0:1], R[:, 1:2], t_vec.reshape(3,1)))
        try:
            self.image_to_ground_homography = np.linalg.inv(H_g2i)
            h_msg = Float64MultiArray()
            h_msg.data = self.image_to_ground_homography.flatten().tolist()
            self.homography_pub.publish(h_msg)
        except np.linalg.LinAlgError:
            self.image_to_ground_homography = None

    def image_callback(self, msg):
        if self.camera_matrix is None or self.image_to_ground_homography is None:
            self.get_logger().warn(
                f'Waiting for data... CamInfo: {self.camera_matrix is not None}, '
                f'Homography: {self.image_to_ground_homography is not None}',
                throttle_duration_sec=2.0
            )
            return

        try:
            # 1. BEV Projection Node
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

            M = np.array([
                [1/self.mpp, 0, self.bev_width / 2],
                [0, -1/self.mpp, self.bev_height],
                [0, 0, 1]
            ])
            final_homography = M @ self.image_to_ground_homography
            bev_image = cv2.warpPerspective(undistorted_image, final_homography, (self.bev_width, self.bev_height))

            # 2. Edge Lane Node
            gray_image = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray_image, self.lane_threshold, 255, cv2.THRESH_BINARY)
            kernel_size = max(1, int(self.morph_kernel_size))
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            mask_morphed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # 3. Skeletonization Node
            skeleton = cv2.ximgproc.thinning(mask_morphed, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)

            # 4. Branch Pruning Node
            pruned_image = prune_branches(skeleton, self.length_threshold)

            # 5. Path Planner Node
            h, w = pruned_image.shape
            robot_x = w // 2
            path_pixels = np.argwhere(pruned_image > 0)
            
            cmd_msg = Twist()
            if len(path_pixels) >= 10:
                target_y_idx = h - self.lookahead_dist
                differences = np.abs(path_pixels[:, 0] - target_y_idx)
                min_idx = np.argmin(differences)
                target_y, target_x = path_pixels[min_idx]

                error_x = robot_x - target_x 
                steering_angle = self.k_p * error_x
                linear_speed = self.max_speed * max(0.1, 1.0 - abs(steering_angle))

                cmd_msg.linear.x = float(linear_speed)
                cmd_msg.angular.z = float(steering_angle)
                
                # Debug Target Publish
                p_msg = PointStamped()
                p_msg.header = msg.header
                p_msg.point.x = float(target_x)
                p_msg.point.y = float(target_y)
                self.target_pub.publish(p_msg)
                self.steering_debug_pub.publish(Float32(data=steering_angle))
            else:
                self.get_logger().warn('No path detected, stopping.', throttle_duration_sec=2.0)

            self.cmd_vel_pub.publish(cmd_msg)

            # Publish Intermediate Representations (For Visualizer Node)
            bev_msg = self.bridge.cv2_to_imgmsg(bev_image, "bgr8")
            bev_msg.header = msg.header
            self.bev_image_pub.publish(bev_msg)

            debug_msg = self.bridge.cv2_to_imgmsg(undistorted_image, "bgr8")
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

            mask_msg = self.bridge.cv2_to_imgmsg(mask_morphed, "mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

            skeleton_msg = self.bridge.cv2_to_imgmsg(skeleton, "mono8")
            skeleton_msg.header = msg.header
            self.skeleton_pub.publish(skeleton_msg)

            pruned_msg = self.bridge.cv2_to_imgmsg(pruned_image, 'mono8')
            pruned_msg.header = msg.header
            self.pruned_skeleton_pub.publish(pruned_msg)

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
