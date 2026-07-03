# slope_drive/visualizer_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        self.bridge = CvBridge()
        
        # 보고 싶은 토픽들을 구독
        self.create_subscription(Image, '/bev/image', self.bev_callback, 10)
        self.create_subscription(Image, '/pruned_skeleton', self.skel_callback, 10)
        
        self.bev_img = None
        self.skel_img = None

        # Create a timer to call show_image periodically
        timer_period = 1.0 / 30.0  # 30 Hz
        self.timer = self.create_timer(timer_period, self.show_image)

    def bev_callback(self, msg):
        self.bev_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # self.show_image() # Removed from here

    def skel_callback(self, msg):
        self.skel_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        # self.show_image() # Removed from here

    def show_image(self):
        if self.bev_img is not None:
            cv2.imshow("BEV Input", self.bev_img)
        else:
            # 데이터가 아직 안 들어왔을 때 대기 화면 표시
            wait_img = np.zeros((400, 400, 3), dtype=np.uint8)
            cv2.putText(wait_img, "Waiting for Topic...", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow("BEV Input", wait_img)
        
        if self.skel_img is not None:
            # 흑백 이미지를 컬러로 변환해서 빨간색으로 표시
            skel_color = cv2.cvtColor(self.skel_img, cv2.COLOR_GRAY2BGR)
            skel_color[self.skel_img > 0] = [0, 0, 255] # Red skeleton
            cv2.imshow("Pruned Skeleton", skel_color)
            
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()