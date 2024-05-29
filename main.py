# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from glob import glob
from itertools import combinations
from math import atan2, sqrt, degrees
import time
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()
        self.count = 0
    def callback(self, data):
        try:
            # listen image topic
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            IMG=img.copy()
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

            def color_detector():
              class ConvexHull:
                def __init__(self, points):
                    self.points = np.array(points)
                    self.vertices = self._graham_scan(self.points)

                def _graham_scan(self, points):
                    def polar_angle(p0, p1):
                        from math import atan2
                        return atan2(p1[1] - p0[1], p1[0] - p0[0])

                    def distance(p0, p1):
                        return (p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2

                    def ccw(p1, p2, p3):
                        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

                    start = min(points, key=lambda p: (p[1], p[0]))

                    sorted_points = sorted(points, key=lambda p: (polar_angle(start, p), distance(start, p)))
                    hull = []
                    for p in sorted_points:
                        while len(hull) > 1 and ccw(hull[-2], hull[-1], p) <= 0:
                            hull.pop()
                        hull.append(p)

                    hull_indices = [np.where((points == h).all(axis=1))[0][0] for h in hull]
                    return np.array(hull_indices)
              def homograph(hsv_image,best_quadrilateral):

                cv2.drawContours(hsv_image, [best_quadrilateral], -1, (255, 0, 0), 5)

                approx = best_quadrilateral.reshape(4, 2)

                width = max(np.linalg.norm(approx[0] - approx[1]), np.linalg.norm(approx[2] - approx[3]))
                height = max(np.linalg.norm(approx[0] - approx[3]), np.linalg.norm(approx[1] - approx[2]))

                dst_points = np.array([
                    [0, 0],
                    [width - 1, 0],
                    [width - 1, height - 1],
                    [0, height - 1]
                ], dtype=np.float32)

                H = cv2.getPerspectiveTransform(np.float32(approx), dst_points)

                warped_image = cv2.warpPerspective(hsv_image, H, (int(width), int(height)))
                return warped_image
              def final():
                  return 'B'
              def find_intersection(line1, line2, img_shape):

                  def line_params(x1, y1, x2, y2):
                      A = y2 - y1
                      B = x1 - x2
                      C = A * x1 + B * y1
                      return A, B, C

                  A1, B1, C1 = line_params(*line1)
                  A2, B2, C2 = line_params(*line2)

                  determinant = A1 * B2 - A2 * B1
                  if determinant == 0:
                      return None

                  x = (B2 * C1 - B1 * C2) / determinant
                  y = (A1 * C2 - A2 * C1) / determinant

                  if 0 <= x < img_shape[1] and 0 <= y < img_shape[0]:
                      return int(round(x)), int(round(y))
                  return None

              def find_all_intersections(lines, img_shape):

                  intersections = set()
                  for line1, line2 in combinations(lines, 2):
                      intersection = find_intersection(line1, line2, img_shape)
                      if intersection:
                          intersections.add(intersection)
                  return list(intersections)

              def extend_line(line, img_shape):

                  x1, y1, x2, y2 = line
                  if x1 == x2:
                      return (x1, 0, x1, img_shape[0] - 1)
                  elif y1 == y2:
                      return (0, y1, img_shape[1] - 1, y1)

                  slope = (y2 - y1) / (x2 - x1)
                  intercept = y1 - slope * x1

                  x_min, y_min = 0, int(intercept)
                  x_max, y_max = img_shape[1] - 1, int(slope * (img_shape[1] - 1) + intercept)
                  y_top, x_top = 0, int(-intercept / slope)
                  y_bottom, x_bottom = img_shape[0] - 1, int((img_shape[0] - 1 - intercept) / slope)

                  points = [
                      (x_min, y_min), (x_max, y_max), (x_top, y_top), (x_bottom, y_bottom)
                  ]
                  points = [(x, y) for x, y in points if 0 <= x < img_shape[1] and 0 <= y <         img_shape[0]]

                  if len(points) >= 2:
                      return points[0][0], points[0][1], points[1][0], points[1][1]
                  else:
                      return None

              def line_distance(line1, line2):

                  def point_line_distance(x0, y0, x1, y1, x2, y2):

                      num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
                      den = sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
                      return num / den

                  x11, y11, x12, y12 = line1
                  x21, y21, x22, y22 = line2

                  distances = [
                      point_line_distance(x11, y11, x21, y21, x22, y22),
                      point_line_distance(x12, y12, x21, y21, x22, y22),
                      point_line_distance(x21, y21, x11, y11, x12, y12),
                      point_line_distance(x22, y22, x11, y11, x12, y12),
                  ]

                  return min(distances)

              def line_angle(line1, line2):

                  def angle(x1, y1, x2, y2):
                      return degrees(atan2(y2 - y1, x2 - x1))

                  angle1 = angle(*line1)
                  angle2 = angle(*line2)
                  angle_diff = abs(angle1 - angle2)
                  return min(angle_diff, 360 - angle_diff)

              def remove_duplicate_lines(lines, min_distance=10, max_angle_difference=10):

                  unique_lines = []

                  for line in lines:
                      is_unique = True
                      for unique_line in unique_lines:
                          if line_distance(line[0], unique_line[0]) < min_distance and line_angle(line[0], unique_line[0]) < max_angle_difference:
                              is_unique = False
                              break
                      if is_unique:
                          unique_lines.append(line)

                  return unique_lines

              def calculate_area(points):

                  x_coords, y_coords = zip(*points)
                  return 0.5 * abs(sum(x_coords[i] * y_coords[(i + 1) % len(points)] - y_coords[i] * x_coords[(i + 1) % len(points)] for i in range(len(points))))

              def sort_points_counterclockwise(points):

                  center_x = sum([p[0] for p in points]) / len(points)
                  center_y = sum([p[1] for p in points]) / len(points)

                  def angle_from_center(point):
                      return atan2(point[1] - center_y, point[0] - center_x)

                  return sorted(points, key=angle_from_center)

              def is_convex_quadrilateral(points):

                  def cross_product(o, a, b):

                      return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

                  points = sort_points_counterclockwise(points)
                  p1, p2, p3, p4 = points

                  return cross_product(p1, p2, p3) * cross_product(p1, p2, p4) >= 0 and \
                      cross_product(p2, p3, p4) * cross_product(p2, p3, p1) >= 0 and \
                      cross_product(p3, p4, p1) * cross_product(p3, p4, p2) >= 0 and \
                      cross_product(p4, p1, p2) * cross_product(p4, p1, p3) >= 0

              def convex_hull_intersections(intersections):

                  if len(intersections) < 3:
                      return intersections

                  hull = ConvexHull(intersections)

                  return [list(intersections[i]) for i in hull.vertices]

              def step():
                  start=time.time()
                  image=IMG.copy()
                  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                  blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                  edges = cv2.Canny(blurred, 50, 150)
                  lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=20)

                  if lines is None:
                      return final()


                  img_shape = image.shape
                  unique_lines = remove_duplicate_lines(lines, min_distance=15, max_angle_difference=10)
                  extended_lines = [extend_line(line[0], img_shape) for line in unique_lines if line is not None]
                  extended_lines = [line for line in extended_lines if line is not None]
                  intersections = find_all_intersections(extended_lines, img_shape)

                  if len(intersections) < 4:
                      return final()

                  convex_intersections = convex_hull_intersections(intersections)

                  max_area = 0
                  best_quadrilateral = None


                  for points in combinations(convex_intersections, 4):
                      if (time.time()-start)>1:
                          return final()
                      if is_convex_quadrilateral(points):
                          sorted_points = sort_points_counterclockwise(points)
                          area = calculate_area(sorted_points)
                          if area > max_area:
                              max_area = area
                              best_quadrilateral = np.array(sorted_points, dtype=np.int32)

                  if best_quadrilateral is not None:
                      mask = np.zeros_like(image, dtype=np.uint8)
                      cv2.fillPoly(mask, [best_quadrilateral], (255, 255, 255))
                      masked_image = cv2.bitwise_and(image, mask)


                      hsv_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)

                      hsv_image=homograph(hsv_image,best_quadrilateral)

                      lower_red1, upper_red1 = (0, 50, 50), (20, 256, 256)
                      lower_red2, upper_red2 = (160, 50, 50), (180, 256, 256)
                      lower_blue, upper_blue = (100, 50, 50), (140, 256, 256)


                      red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
                      red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
                      blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)


                      red_mask = cv2.bitwise_or(red_mask1, red_mask2)

                      h, w, _=hsv_image.shape
                      red_sum = np.sum(red_mask > 0)
                      blue_sum = np.sum(blue_mask > 0)
                      other_sum = h*w-red_sum-blue_sum


                      #print(red_sum, blue_sum, other_sum)
                      rgb_sums = {'R': red_sum, 'O': other_sum, 'B': blue_sum}
                      most_dominant_color = max(rgb_sums, key=rgb_sums.get)

                      return most_dominant_color

                  else:
                      return final()

              return step()

            
            most_dominant_color=color_detector()
            if most_dominant_color=='B':
              print("B")
              msg.frame_id='+1'
            elif most_dominant_color=='R':          
              print("R")
              msg.frame_id='-1'
            else :
              print("O")
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
