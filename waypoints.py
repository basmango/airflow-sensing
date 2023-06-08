#!/usr/bin/env python

#import necessary packages
import rospy
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GlobalPositionTarget

#error function using haversine formula
def error(x_expected, y_expected, x_real, y_real):
    R = 6371000
    x_expected = math.radians(x_expected)
    y_expected = math.radians(y_expected)
    x_real = math.radians(x_real)
    y_real = math.radians(y_real)
    dlon = x_real - x_expected
    dlat = y_real - y_expected
    a = math.sin(dlat/2)**2 + math.cos(y_expected) * math.cos(y_real) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return d

#callback function
def callback(msg):
    global satfix
    global waypoints
    global current_waypoint_index

    print('current position :')
    print(msg.latitude, msg.longitude)
    print('current waypoint :')
    print(waypoints[current_waypoint_index][0], waypoints[current_waypoint_index][1])

    geopose_stamped = GlobalPositionTarget()
    geopose_stamped.header.stamp = rospy.Time.now()
    geopose_stamped.header.frame_id = 'map'
    geopose_stamped.latitude = waypoints[current_waypoint_index][0]
    geopose_stamped.longitude = waypoints[current_waypoint_index][1]
    geopose_stamped.altitude = msg.altitude
     
    
    # use haversine to calculate distance between current position and current waypoint

    print('distance from the next waypoints :')
    print(error(waypoints[current_waypoint_index][0], waypoints[current_waypoint_index][1], msg.latitude, msg.longitude))

    if error(waypoints[current_waypoint_index][0], waypoints[current_waypoint_index][1], msg.latitude, msg.longitude) < 1:    


        if current_waypoint_index != len(waypoints) - 1:
            current_waypoint_index = (current_waypoint_index + 1) % len(waypoints)

    pub_obj.publish(geopose_stamped)


#initialize the node
rospy.init_node('square')

#subscribe to global_position/global
sub_obj = rospy.Subscriber('/mavros/global_position/global', NavSatFix, callback)

#publisher object
pub_obj = rospy.Publisher('/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)

rate = rospy.Rate(10)
posemsg = PoseStamped()

waypoints=[(23.28884, 77.27397), (23.28886, 77.27417), (23.28899, 77.27416), (23.28897, 77.27396), (23.2891, 77.27394), (23.28911, 77.27414)]

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
