#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
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
    rospy.loginfo(" init class.")

    self.image_grey_pub = rospy.Publisher("/image_processing/grey_img", Image, queue_size=1)
    self.image_bin_pub = rospy.Publisher("/image_processing/bin_img", Image, queue_size=1)
    self.image_dot_pub = rospy.Publisher("/image_processing/dot_img", Image, queue_size=1)
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
    self.image_shape=[None,None]
    self.ransac_treshold=30
    self.numberOfIterations=10

  def solve(self, img_points):
    img_points=np.array(img_points)
    fx = 614.1699#1367
    fy = 614.9002#1367
    cx = 329.9491
    cy = 237.2788
    camera_mat = np.zeros((3, 3, 1))
    camera_mat[:, :, 0] = np.array([[fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]])
    k1 = 0.1115
    k2 = -0.1089
    p1 = 0
    p2 = 0
    dist_coeffs = np.zeros((4, 1))
    dist_coeffs[:, 0] = np.array([[k1, k2, p1, p2]])
    # far to close, left to right (order of discovery) in cm
    obj_points = np.zeros((6, 3, 1))
    obj_points[:, :, 0] = np.array([[58.5, 0.0, 0],
          [58.5, 39.8, 0],
          [28.0, 0.0, 0],
          [29.0, 40.0, 0],
          [0.0, 0.0,  0],
          [1.5, 40.0, 0]])
    rospy.loginfo("image:"+str(img_points))
    rospy.loginfo("object:"+str(obj_points[:, :, 0]))
    _, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_mat, dist_coeffs)
    rmat = np.zeros((3, 3))
    rotation_matrix=cv2.Rodrigues(rvec, rmat, jacobian=0)[0]
    return (rvec, tvec, rotation_matrix)

  def getMb(self, vec0, vec1):
    m = (vec1[1]-vec0[1]) / (vec1[0]-vec0[0])
    b = lambda x: int((x-vec0[0]) * m + vec0[1])
    return (m,b(0),b)

  def drawLine(self, img, p):
    p0 = (p[0][1], p[0][0])
    p1 = (p[1][1], p[1][0])
    rospy.loginfo("Choice:\np0 = (%f, %f)\p1=(%f, %f)" % (p0[0], p0[1], p1[0], p1[1]))
    return cv2.line(img, p0, p1, (255, 255, 255))


  def ransac(self, vectors, threshold):
    results=np.zeros((self.numberOfIterations,3))
    for i in range(self.numberOfIterations):
    
      choice = vectors[np.random.choice(range(vectors.shape[0]), 2, replace=False)]
      rospy.loginfo(choice[0] in vectors)
      rospy.loginfo(choice[0] in vectors)
      rospy.loginfo("choice:"+str(choice))
      m, b, b_func = self.getMb(choice[0], choice[1])
      #m,b,bf = self.getMb(p[0],p[1])
         #rospy.loginfo("m = %f; b = %f" % (m, b))

      inliers=np.zeros(vectors.shape[0])
      for vec in range(len(vectors)):
        if abs(vectors[vec,1]-b_func(vectors[vec,0]))<=self.ransac_treshold:
          inliers[vec]=1
      results[i,:]=[m,b,sum(inliers)]
      #rospy.loginfo("   m = %f; b = %f" % (m, b))
    best_result=results[np.argmax(results[:,2]),:]
    rospy.loginfo(str(results))
    rospy.loginfo("  best result"+str(best_result))

    m,b,_=best_result 
    edge=[None,None]
    edge[0]=(0, int(b))
    edge[1]=(self.image_shape[0], int(m*(self.image_shape[0])+b))
      
    return edge

    
  def callback(self,data):
    rospy.loginfo("\n\t::: start callback :::")

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #make it gray
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    self.image_shape=gray.shape

    try:
      self.image_grey_pub.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
      rospy.loginfo("publishing grey image.")
    except CvBridgeError as e:
      print(e)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 200#245
    ret, img_binary = cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)

    #new----
    #rospy.loginfo("dim thres1 ret"+str(thresh1.shape)+str(ret))
    dim = img_binary.shape

    #img_binary[0:int(0.3*dim[0])] = 0 #Car 122
    #img_binary[int(0.93*dim[0]):dim[0], :] = 0 #Car 122
    img_binary[0:int(0.3*dim[0])] = 0 #Car 125
    img_binary[int(0.85*dim[0]):dim[0], :] = 0 #Car 125

    #try:
     # self.image_bin_pub.publish(self.bridge.cv2_to_imgmsg(img_binary, "mono8"))
      #rospy.loginfo("publishing binary image.")
    #except CvBridgeError as e:
      #print(e)

    vectors = np.array(np.nonzero(img_binary)).T
    rospy.loginfo("vectors"+str(vectors.shape))
    vectors = np.float32(vectors)

    #test_img = np.zeros(img_binary.shape)
    #for x in vectors:
    #  test_img[int(x[0]), int(x[1])] = 255
    #test_img = test_img.astype(np.uint8)

    p = self.ransac(vectors, 5)
    
    img_line = self.drawLine(img_binary, p)

    try:
      self.image_bin_pub.publish(self.bridge.cv2_to_imgmsg(img_binary, "mono8"))
      rospy.loginfo("publishing binary image + line.")
    except CvBridgeError as e:
      print(e)    

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
