#!/usr/bin/env python


import rospy
import geometry_msgs.msg
import std_msgs.msg


L=33*10**(-2)
R=20*10**(-2)
wl=0
wr=0
leftWheelCommand=rospy.Publisher('vrep/leftWheelCommand',std_msgs.msg.Float64, queue_size=1)
rightWheelCommand=rospy.Publisher('vrep/rightWheelCommand',std_msgs.msg.Float64, queue_size=1)

def convert(msg):
    v=msg.linear.x
    w=msg.angular.z
    global wl
    global wr
    wl=(v-w*L/2)/R
    wr=(v+w*L/2)/R
    rospy.loginfo("I heard %s", wl)
    rospy.loginfo("I heard then %s", wr)


    leftWheelCommand.publish(std_msgs.msg.Float64(wl))
    rightWheelCommand.publish(std_msgs.msg.Float64(wr))

def controler():
    rospy.init_node('controler', anonymous=True)
    hellow_world="hellow_world"
    rospy.loginfo(hellow_world)
    rate=rospy.Rate(1)
    rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, convert)

    rospy.spin()



if __name__=='__main__':
    controler()


