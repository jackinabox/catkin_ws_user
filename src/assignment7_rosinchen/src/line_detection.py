#!/usr/bin/env python
import time
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#import matplotlib
#matplotlib.use('Agg')
#from matplotlib import pyplot as plt

# from __future__ import print_function

class image_converter:

  def __init__(self):
    #rospy.loginfo(" init class.")

    self.image_lines_pub = rospy.Publisher("/image_processing/line_img", Image, queue_size=1)
    self.image_grey_pub = rospy.Publisher("/image_processing/grey_img", Image, queue_size=1)
    self.image_bin_pub = rospy.Publisher("/image_processing/bin_img", Image, queue_size=1)
    self.pub_line_param= rospy.Publisher("/line_parameter", Point, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
    self.image_shape = [None, None]
    self.ransac_threshold = 8
    self.numberOfIterations = 20
    self.samplerate = 0.025
    self.valid_line_threshold = 0.5
    self.pub_type = {0: "mono8", 1: "bgr8"}
    self.eps = np.finfo(float).eps

  def pub_param(self, m, b):
    rospy.loginfo("publishing line params...")
    my_point = Point(m, b, 0)
    self.pub_line_param.publish(my_point)

  def get_m_b(self, vec0, vec1):
    m = (vec1[1]-vec0[1]) / (vec1[0]-vec0[0]+self.eps)
    b = lambda x: int((x-vec0[0]) * m + vec0[1])
    return m, b(0), b

  def drawLine(self, img, p, color, thickness):
    p0 = (p[0][1], p[0][0])
    p1 = (p[1][1], p[1][0])
    #rospy.loginfo("Choice:\np0 = (%f, %f)\np1=(%f, %f)" % (p0[0], p0[1], p1[0], p1[1]))
    cv2.line(img, p0, p1, color, thickness)

  def print_points(self, points):
    test_img = np.zeros(self.image_shape)
    for p in points:
      test_img[int(p[0]), int(p[1])] = 255
    return test_img.astype(np.uint8)

  def pub_img(self, publisher, img, log, img_type):
    try:
      publisher.publish(self.bridge.cv2_to_imgmsg(img, self.pub_type[img_type]))
      #rospy.loginfo(log)
    except CvBridgeError as e:
      print(e)

  def getPointsFromLine(self, line):
    m, b = line
    edge = [None, None]
    edge[0] = (0, int(b))
    edge[1] = (self.image_shape[0], int(m * (self.image_shape[0]) + b))
    return edge

  def ransac(self, points):
    rospy.loginfo("points_dim: %s" % str(points.shape))
    results = np.zeros((self.numberOfIterations, 3))
    inliersOfIterations = {}
    for iter in range(self.numberOfIterations):
      choice = points[np.random.choice(range(points.shape[0]), 2, replace=False)]
      #rospy.loginfo("choice:" + str(choice))
      m, b, b_func = self.get_m_b(choice[0], choice[1])

      inliers = np.zeros(points.shape[0])
      for index, point in enumerate(points):
        if abs(point[1] - b_func(point[0])) <= self.ransac_threshold:
          inliers[index] = 1
      inliersOfIterations[iter] = inliers
      results[iter, :] = [m, b, sum(inliers)]
      #rospy.loginfo("   m = %f; b = %f" % (m, b))
    indBest = np.argmax(results[:, 2])
    best_result = results[indBest, :]
    m, b, _ = best_result
    return (m, b), inliersOfIterations[indBest]

  def line_is_valid(self, inliers):
    return np.sum(inliers) / len(inliers) >= self.valid_line_threshold

  def callback(self, data):
    #rospy.loginfo("\n\t::: start callback :::")

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)




    # make it gray
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    self.image_shape = gray.shape

    #self.pub_img(self.image_grey_pub, gray, "published gray image", 0)

    # make it binary
    bi_gray_max = 255
    bi_gray_min = 200#245
    _, img_binary = cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)
    dim = img_binary.shape

    #img_binary[0:int(0.3*dim[0])] = 0 #Car 122
    #img_binary[int(0.93*dim[0]):dim[0], :] = 0 #Car 122
    img_binary[0:int(0.3*dim[0])] = 0 #Car 125
    img_binary[int(0.85*dim[0]):dim[0], :] = 0 #Car 125

    #self.pub_img(self.image_bin_pub, img_binary, "published binary image", 0)

    vectors = np.float32(np.array(np.nonzero(img_binary)).T)
    #rospy.loginfo("vectors"+str(vectors.shape))
    vectors = vectors[np.random.choice(range(vectors.shape[0]), int(vectors.shape[0] * self.samplerate), replace=False)]

    '''
    point_img = self.print_points(vectors)
    self.pub_img(self.image_bin_pub, point_img, "published initial binary image", 0)
    '''
    start_time = time.time()
    line_one, inliers_one = self.ransac(vectors)
    #rospy.loginfo(inliers)
    #vectors_subset = vectors[inliers_one == 0]

    #point_img = self.print_points(vectors_subset)
    #self.pub_img(self.image_bin_pub, point_img, "published reduced binary image", 0)

    #line_two, _ = self.ransac(vectors_subset)

    # print m & b for lines
    #rospy.loginfo("\n\tline one: m = %f,  b = %f\n\tline two: m = %f,  b = %f" %
    #              (line_one[0], line_one[1], line_two[0], line_two[1]))

    print("--- %s seconds ---" % (time.time() - start_time))

    if self.line_is_valid(inliers_one):
      self.pub_param(line_one[0], line_one[1])
      rospy.loginfo("%s: line params: m = %f,  b = %f" %
                    (rospy.get_caller_id(), line_one[0], line_one[1]))
      '''
      p_one = self.getPointsFromLine(line_one)
      #p_two = self.getPointsFromLine(line_two)

      red = (0, 0, 255)
      green = (0, 255, 0)
      color = green
      thickness = 3
      self.drawLine(cv_image, p_one, color, thickness)
      #self.drawLine(cv_image, p_two, color, thickness)

      self.pub_img(self.image_lines_pub, cv_image, "published image + lines", 1)
      '''


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
