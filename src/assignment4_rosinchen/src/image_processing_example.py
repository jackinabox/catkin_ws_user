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

    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    rospy.loginfo("  start callback.")

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 245
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    #new----
    #rospy.loginfo("dim thres1 ret"+str(thresh1.shape)+str(ret))
    dim=thresh1.shape
    thresh1=thresh1[0:int(0.85*dim[0]),:]

    vectors=np.array(np.nonzero(thresh1)).T
    rospy.loginfo("vectors"+str(vectors.shape))
    vectors = np.float32(vectors)

    pointer=1
    epsilon=20
    clusterPointer=np.zeros(vectors.shape[0])
    for vect in range(len(vectors)):
      for vect2 in range(vect,len(vectors)):
        if np.linalg.norm(vectors[vect]-vectors[vect2])<epsilon:
          if clusterPointer[vect]!=0:
            clusterPointer[vect2]=clusterPointer[vect]
          else:
            clusterPointer[vect]=pointer
            clusterPointer[vect2]=pointer
            pointer+=1

    #rospy.loginfo("clusterPointer"+str(clusterPointer))
    clusterVectors=[None for x in range(int(max(clusterPointer)))]
    means=[None for x in range(int(max(clusterPointer)))]
    for cluster in range(int(max(clusterPointer))):
      clusterVectors[cluster]=vectors[clusterPointer==cluster+1]
      means[cluster]=np.mean(vectors[clusterPointer==cluster+1],axis=0)

    #rospy.loginfo("clusterPointer"+str(clusterVectors))
    #rospy.loginfo("clusterPointer mean"+str(means))
    
    thresh1[:,:]=0 #w
    for i in range(int(max(clusterPointer))):
      thresh1[int(means[i][0]),int(means[i][1])]=255 #b
    

    #rospy.sleep(1)




    #ret,labels,centers=cv2.kmeans(vectors, 6, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,1,10),
    #       attempts=10, bestLabels=None, flags=cv2.KMEANS_RANDOM_CENTERS)
    #centers=np.array(centers,dtype=int)
    #rospy.loginfo("   centers:"+str(centers)+"shape"+str(centers.shape))
    #rospy.loginfo(thresh1.shape)
    
    #thresh1[:,:]=0 #w
    #for i in range(6):
    #  thresh1[centers[i,0],centers[i,1]]=255 #b
    
    #rospy.sleep(2)

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

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
      rospy.loginfo("  publishing an image.")
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
