#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

carID = 5
location = np.array([])
location_corr = np.array([])

laneID = 1
distance = 0.5

# Model. all in cm
center_upper = [196, 215.5]
center_lower = [405, 215.5]
lines_inner = [[[196, 79], [405, 79]], [[196, 352], [405, 352]]]
lines_outer = [[[196, 47], [405, 47]], [[196, 384], [405, 384]]]
# lines = [left line, right line] with left line = [upper point, lower point]
radius_inner = 136.5
radius_outer = 168.5

twoLines = np.array((lines_inner, lines_outer)) / 100.  # convert to meter
twoRadii = np.array((radius_inner, radius_outer)) / 100.
twoCenters = np.array((center_upper, center_lower)) / 100.


def nearest_point(given_point, laneID=laneID):
    x, y = given_point
    lines = twoLines[laneID - 1]
    radius = twoRadii[laneID - 1]
    centers = twoCenters

    x, y = given_point
    if x < centers[0, 0]:  # upper semicircle
        vec_center_given = given_point - centers[0]
        vec_circle = vec_center_given * radius / np.linalg.norm(vec_center_given)
        return 0, np.array(centers[0] + vec_circle)

    elif x > centers[1, 0]:  # lower semicircle
        vec_center_given = given_point - centers[1]
        vec_circle = vec_center_given * radius / np.linalg.norm(vec_center_given)
        return 1, np.array(centers[1] + vec_circle)

    elif y <= centers[0, 1]:  # left line
        return 2, np.array([given_point[0], lines[0, 0, 1]])

    elif y > centers[0, 1]:  # right line
        return 3, np.array([given_point[0], lines[1, 0, 1]])

    else:
        print("ERROR in choice of track part!")


def look_ahead(given_point, laneID, distance):
    distance_corr = distance + 0.3
    case, nearPoint = nearest_point(given_point, laneID)
    # print("nearest_point:  ",nearPoint)
    phi = 0.  # np.pi()/10. # positive when parallel to x-axis
    centers = twoCenters
    if case == 0:  # upper semicircle
        vec_center_given = nearPoint - centers[0]
        vec_circle = vec_center_given / np.linalg.norm(vec_center_given)
        distance_rotated = np.array([-distance_corr * vec_circle[1], distance_corr * vec_circle[0]])
        # print("distance_corr, rotated",distance_corr,distance_rotated)
        raw_ahead = nearPoint + distance_rotated
        return nearest_point(raw_ahead)[1]
    elif case == 1:  # lower semicircle
        vec_center_given = nearPoint - centers[1]
        vec_circle = vec_center_given / np.linalg.norm(vec_center_given)
        distance_rotated = np.array([-distance_corr * vec_circle[1], distance_corr * vec_circle[0]])
        # print("distance_corr, rotated",distance_corr,distance_rotated)
        raw_ahead = nearPoint + distance_rotated
        return nearest_point(raw_ahead)[1]

    # phi = None
    # distance_rotated = np.array([distance_corr*np.cos(phi), distance_corr*np.sin(phi)])
    # print("distance_corr, rotated",distance_corr,distance_rotated)
    # raw_ahead = nearPoint + distance_rotated
    # return nearest_point(raw_ahead)[1]

    elif case == 2:  # left line, driving downward, x increasing
        raw_ahead = np.array([nearPoint[0] + distance_corr, nearPoint[1]])
        return nearest_point(raw_ahead)[1]
    elif case == 3:  # right line, driving upward, x decreasing
        raw_ahead = np.array([nearPoint[0] - distance_corr, nearPoint[1]])
        return nearest_point(raw_ahead)[1]


def callback(data):
    global location
    global location_corr
    x, y = data.pose.pose.position.x, data.pose.pose.position.y
    print("position:       ", x, y)
    p_ahead = look_ahead((x, y), laneID, distance)
    print("p_ahead", p_ahead)
    toPub = Point(p_ahead[0], p_ahead[1], 0)
    # print("ahead:          ",toPub.x,toPub.y)
    # print("    ------------------")
    pub_target.publish(toPub)


# location = np.append(location, ([x, y]))
# location_corr = np.append(location_corr, (look_ahead((x, y),laneID)))
# rospy.loginfo("x,y:",data)
# print(location)
# np.save("location.npy", location)
# np.save("nearest_point.npy", location_corr)
# rospy.sleep(1)

rospy.init_node("localize", anonymous=True)

# publish topic /target_point

# create subscribers and publishers
pub_target = rospy.Publisher("/target_point", Point, queue_size=1)
sub_pos = rospy.Subscriber("/localization/odom/" + str(carID), Odometry, callback, queue_size=1)
# print("hello")

rospy.spin()
