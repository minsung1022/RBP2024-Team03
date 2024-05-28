# !/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/color', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()
        self.count = 0
    def callback(self, data):
        try:
            # listen image topic
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # prepare rotate_cmd msg
            # DO NOT DELETE THE BELOW THREE LINES!
            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # default: STOP
    
            # determine background color
            # TODO 
            # determine the color and assing +1, 0, or, -1 for frame_id
            # msg.frame_id = '+1' # CCW 
            # msg.frame_id = '0'  # STOP
            # msg.frame_id = '-1' # CW 
            
            img = cv2.imread(filename)
            h, w, _ = img.shape
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


            black = cv2.inRange(hsv_img, (0, 0, 0), (180, 20, 70))

            contours, _ = cv2.findContours(black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            if not contours:
                return None, rgb_img

            index = 0
            possible = True
            approx = None
            while True:
                if index >= len(contours):
                    index = 0
                    possible = False
                    break
                cc = contours[index]
                epsilon = 0.02 * cv2.arcLength(cc, True)
                approx = cv2.approxPolyDP(cc, epsilon, True)

                if len(approx) == 4:
                    break
                else:
                    index += 1

            if approx is None or len(approx) != 4:
                return None, rgb_img

            cv2.drawContours(rgb_img, [approx], -1, (255, 0, 0), 5)
            approx = approx.reshape(4, 2)
            width = max(np.linalg.norm(approx[0] - approx[1]), np.linalg.norm(approx[2] - approx[3]))
            height = max(np.linalg.norm(approx[0] - approx[3]), np.linalg.norm(approx[1] - approx[2]))

            dst_points = np.array([
                [0, 0],
                [width - 1, 0],
                [width - 1, height - 1],
                [0, height - 1]
            ], dtype=np.float32)

            H = cv2.getPerspectiveTransform(np.float32(approx), dst_points)

            warped_image = cv2.warpPerspective(img, H, (int(width), int(height)))

            hsv_warped_image = cv2.cvtColor(warped_image, cv2.COLOR_BGR2HSV)


            lower_red1, upper_red1 = (0, 50, 50), (10, 256, 256)
            lower_red2, upper_red2 = (170, 50, 50), (180, 256, 256)
            lower_green, upper_green = (50, 50, 50), (70, 256, 256)
            lower_blue, upper_blue = (110, 50, 50), (130, 256, 256)


            red_mask1 = cv2.inRange(hsv_warped_image, lower_red1, upper_red1)
            red_mask2 = cv2.inRange(hsv_warped_image, lower_red2, upper_red2)
            green_mask = cv2.inRange(hsv_warped_image, lower_green, upper_green)
            blue_mask = cv2.inRange(hsv_warped_image, lower_blue, upper_blue)


            red_mask = cv2.bitwise_or(red_mask1, red_mask2)


            red_sum = np.sum(red_mask > 0)
            green_sum = np.sum(green_mask > 0)
            blue_sum = np.sum(blue_mask > 0)

            rgb_sums = {'R': red_sum, 'G': green_sum, 'B': blue_sum}
            most_dominant_color = max(rgb_sums, key=rgb_sums.get)
            
            if most_dominant_color=='B':
              msg.frame_id='+1'
            elif most_dominant_color=='R':
              msg.frame_id='-1'
            else :
              msg.frame_id='0'
            # publish color_state
            self.color_pub.publish(msg)
            
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % e)


if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
    
