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
    b = lambda x: (x-vec0[0]) * m + vec0[1]
    return (m,b(0),b)

  def drawLine(self, img, p):
    p0 = (p[0][1], p[0][0])
    p1 = (p[1][1], p[1][0])
    rospy.loginfo("Choice:\np0 = (%f, %f)\p1=(%f, %f)" % (p0[0], p0[1], p1[0], p1[1]))
    return cv2.line(img, p0, p1, (255, 255, 255))

  def ransac(self, vectors, threshold):

    choice = vectors[np.random.choice(range(vectors.shape[0]), 10, replace=False)]
    rospy.loginfo(choice[0] in vectors)
    rospy.loginfo(choice[0] in vectors)
    rospy.loginfo("choice:"+str(choice))
    m, b, b_func = self.getMb(choice[0], choice[1])
    rospy.loginfo("m = %f; b = %f" % (m, b))
    return choice

    
  def callback(self,data):
    rospy.loginfo("\n\t::: start callback :::")

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #make it gray
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

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

    test_img = np.zeros(img_binary.shape)
    for x in vectors:
      test_img[int(x[0]), int(x[1])] = 255
    test_img = test_img.astype(np.uint8)

    p = self.ransac(vectors, 5)
    img_line = self.drawLine(img_binary, p)

    try:
      self.image_bin_pub.publish(self.bridge.cv2_to_imgmsg(img_binary, "mono8"))
      rospy.loginfo("publishing binary image + line.")
    except CvBridgeError as e:
      print(e)

    """

    numberOfClusters=1
    epsilon = 20  # in pixels
    clusterIndices = np.zeros(vectors.shape[0])
    for vec_x in range(len(vectors)):
      for vec_y in range(vec_x, len(vectors)):
        if np.linalg.norm(vectors[vec_x]-vectors[vec_y]) < epsilon:
          if clusterIndices[vec_x] != 0:
            clusterIndices[vec_y] = clusterIndices[vec_x]
          else:
            clusterIndices[vec_x] = numberOfClusters
            clusterIndices[vec_y] = numberOfClusters
            numberOfClusters += 1

    #rospy.loginfo("clusterIndices"+str(clusterIndices))
    #clusterVectors=[None for x in range(int(max(clusterIndices)))]

    means = [None for x in range(int(max(clusterIndices)))]
    for cluster in range(int(max(clusterIndices))):
      #clusterVectors[cluster]=vectors[clusterPointer==cluster+1]
      means[cluster] = np.mean(vectors[clusterIndices == cluster+1], axis=0)
      #rospy.loginfo("cluster "+str(cluster)+": "+str(means[cluster][:]))

    #rospy.loginfo("means"+str(means))
    #rospy.loginfo("clusterPointer mean"+str(means))

    img_binary[:, :] = 0
    for i in range(int(max(clusterIndices))):
      img_binary[int(means[i][0]), int(means[i][1])] = 255

    try:
      self.image_dot_pub.publish(self.bridge.cv2_to_imgmsg(img_binary, "mono8"))
      rospy.loginfo("publishing centroids image.")
    except CvBridgeError as e:
      print(e)

    rvec, tvec, rotation_matrix = self.solve(means)
    rospy.loginfo("\nrvec,tvec:\n"+str(rvec)+"\n"+str(tvec))
    rospy.loginfo("rotation matrix:\n"+str(rotation_matrix))
    transformation=np.zeros((4, 4))
    transformation[0:3, 0:3] = rotation_matrix
    transformation[0:3, 3] = tvec[:, 0]
    transformation[3, 3] = 1
    rospy.loginfo("\ntransformation matrix:\n"+str(transformation))
    transformation_inv = np.linalg.inv(transformation)
    rospy.loginfo("\ntransformation matrix INV:\n"+str(transformation_inv))
    #t_vec_real=np.matmul(transformation_inv,np.append(tvec[:,0],0))
    #vec_real = np.dot(transformation_inv, np.append([406.82318115, 583.26831055, 0], 1))
    vec_real = np.dot(transformation_inv, np.append([0., 0., 0.], 1))
    rospy.loginfo("vec real:"+str(vec_real))
    """

    """
    #gauss
    MAX_KERNEL_LENGTH = 2;
    i= 5
    dst=cv2.GaussianBlur(cv_image,(5,5),0,0)

    #edge
    dx = 1;
    dy = 1;
    ksize = 3; #1,3,5,7
    scale = 1
    delta = 0
    edge_img=cv2.Sobel(thresh1, cv2.CV_8UC1, dx, dy, ksize, scale, delta, cv2.BORDER_DEFAULT)
    
    #bi_rgb
    r_max = 244;
    r_min = 0;
    g_max = 255;
    g_min = 0;
    b_max = 255;
    b_min = 0;
    b,g,r = cv2.split(cv_image)
    for j in range(cv_image.shape[0]):
      for i in range(cv_image.shape[1]):
        if (r[j,i] >= r_min and r[j,i] <= r_max):
          if (g[j,i] >= g_min and g[j,i] <= g_max):
            if (b[j,i] >= b_min and b[j,i] <= b_max):
              #pass
              r[j,i]=0
              g[j,i]=0
              b[j,i]=0
            else:
              #pass
              r[j,i]=255
              g[j,i]=255
              b[j,i]=255
    bi_rgb = cv2.merge((b,g,r))
    
    #bi_hsv
    h_max = 255;
    h_min = 0;
    s_max = 255;
    s_min= 0;
    v_max = 252;
    v_min = 0;
    hsv=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV);
    h,s,v = cv2.split(hsv)

    for j in xrange(hsv.shape[0]):
      for i in xrange(hsv.shape[1]):
        if  (v[j,i]>= v_min and v[j,i]<= v_max and s[j,i]>= s_min and s[j,i]<= s_max and h[j,i]>= h_min and h[j,i]<= h_max):
          #pass
          h[j,i]=0
          s[j,i]=0
          v[j,i]=0
        else:
          #pass
          h[j,i]=255
          s[j,i]=255
          v[j,i]=255

    bi_hsv = cv2.merge((h,s,v))
    """
    # titles = ['Original Image', 'GRAY','BINARY','GAUSS','EDGE','BI_RGB','BI_HSV']
    # images = [cv_image, gray, thresh1,dst,edge_img,bi_rgb,bi_hsv]
    #
    # for i in xrange(7):
    #   plt.subplot(2,4,i+1),plt.imshow(images[i],'gray')
    #   plt.title(titles[i])
    #   plt.xticks([]),plt.yticks([])
    #
    # plt.show()
    # print("Done")
      

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
