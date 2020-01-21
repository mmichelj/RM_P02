import numpy as np
from scipy.spatial import ConvexHull

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node('hoge')

pub = rospy.Publisher('/marker', Marker, queue_size=1)

marker = Marker()
marker.header.frame_id = 'odom'
marker.color.a = 1.0;
marker.color.g = 1.0;
marker.type = Marker.CUBE_LIST
marker.scale.x = 0.01
marker.pose.orientation.w = 1.0

N = 30
while not rospy.is_shutdown():

    marker.header.stamp = rospy.Time.now()
    marker.points = []
    
    marker.points.append(Point(0, 0, 0))
    marker.points.append(Point(1, 0, 0))
    marker.points.append(Point(1, 1, 0))
    marker.points.append(Point(0, 1, 0))
    marker.points.append(Point(0, 0, 1))
    marker.points.append(Point(1, 0, 1))
    marker.points.append(Point(1, 1, 1))
    marker.points.append(Point(0, 1, 1))
    pub.publish(marker)
    rospy.sleep(2.0)