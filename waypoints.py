#!/usr/bin/env python

#import necessary packages
import rospy
import math
import time 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GlobalPositionTarget
from vincenty import vincenty
from geographic_msgs.msg import GeoPoseStamped

#error function using vincenty formula
def distance(x_expected, y_expected, x_real, y_real):
    return vincenty((x_expected, y_expected), (x_real, y_real)) * 1000

#callback function
def callback(msg):
    global satfix
    global waypoints
    global current_waypoint_index

    print('current position :')
    print(msg.latitude, msg.longitude)
    print('current waypoint :')
    print(waypoints[current_waypoint_index][0], waypoints[current_waypoint_index][1])

    geopose_stamped = GeoPoseStamped()
    geopose_stamped.header.stamp = rospy.Time.now()
    geopose_stamped.header.frame_id = 'map'
    geopose_stamped.pose.position.latitude = waypoints[current_waypoint_index][0]
    geopose_stamped.pose.position.longitude = waypoints[current_waypoint_index][1]
    geopose_stamped.pose.position.altitude = msg.altitude
     
    
    # use haversine to calculate distance between current position and current waypoint

    print('distance from the next waypoints :')
    print(distance(waypoints[current_waypoint_index][0], waypoints[current_waypoint_index][1], msg.latitude, msg.longitude))

    if distance(waypoints[current_waypoint_index][0], waypoints[current_waypoint_index][1], msg.latitude, msg.longitude) < 1:    


        if current_waypoint_index != len(waypoints) - 1:
            current_waypoint_index = (current_waypoint_index + 1) % len(waypoints)

        time.sleep(30)

    pub_obj.publish(geopose_stamped)


#initialize the node
rospy.init_node('square')

#subscribe to global_position/global
sub_obj = rospy.Subscriber('/mavros/global_position/global', NavSatFix, callback)

#publisher object
pub_obj = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)

rate = rospy.Rate(10)
posemsg = PoseStamped()

waypoints=[(23.28884098,77.27393332), (23.28883891,77.27402810), ( 23.28884098,77.27412853), (23.28884305,77.27422443),
           (23.28893114,77.27422443),( 23.28893218,77.27412740),(  23.28893218,77.27402810),(23.28893114,77.27393107),
           ( 23.28901924,77.27393220),( 23.28901924,77.27402810),(23.28901820,77.27412853),(23.28901924,77.27422330)]

# Define waypoints in GPS format [latitude, longitude, altitude]
"""
waypoints = [
    [23.28884063, 77.27393580],
    [23.28884507, 77.27402600],
    [23.28888783, 77.27402600],
    [23.28888783, 77.27392838],
    [23.28893336, 77.27392838],
    [23.28893336, 77.27402750],
    [23.28898026, 77.27402750],
    [23.28898026, 77.27392838],
    [23.28902440, 77.27392838],
    [23.28902440, 77.27402750]
]
"""
current_waypoint_index = 0

rospy.spin()
