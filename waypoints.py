#!/usr/bin/env python

#import necessary packages
import rospy
import math
from geometry_msgs.msg import PoseStamped

#error function
def error(x_expected, y_expected, x_real, y_real):
    return math.sqrt(((x_real - x_expected)**2) + ((y_real - y_expected)**2))

#callback function
def callback(msg): 

    print ('distance from the next waypoint :')
    print(error(posemsg.pose.position.x, posemsg.pose.position.y, msg.pose.position.x, msg.pose.position.y))

    #arbitrary point 1 (0,0) 
    if error(0, 0, msg.pose.position.x, msg.pose.position.y) < 2:

        #update the position setpoint as (50,0)
        posemsg.pose.position.x = 50
        posemsg.pose.position.y = 0
        posemsg.pose.position.z = 10

    #arbitrary point 2 (50,0)        
    if error(50, 0, msg.pose.position.x, msg.pose.position.y) < 2:

        #update the position setpoint as (50,50)
        posemsg.pose.position.x = 50
        posemsg.pose.position.y = 50
        posemsg.pose.position.z = 10
      
    #arbitrary point 3 (50,50)
    if error(50, 50, msg.pose.position.x, msg.pose.position.y) < 2:

        #update the position setpoint as (0,50)
        posemsg.pose.position.x = 0
        posemsg.pose.position.y = 50
        posemsg.pose.position.z = 10

    #arbitrary point 4 (0,50)      
    if error(0, 50, msg.pose.position.x, msg.pose.position.y) < 2:

        #update the position setpoint as (0,0)
        posemsg.pose.position.x = 0
        posemsg.pose.position.y = 0
        posemsg.pose.position.z = 10

    #publish the position setpoint 
    pub_obj.publish(posemsg)


#initialize the node
rospy.init_node('square')

#subscribe to local_position/setpoint
sub_obj = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback) 

#publisher object
pub_obj = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

rate = rospy.Rate(10)
posemsg = PoseStamped()
rospy.spin()