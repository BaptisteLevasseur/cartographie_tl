#!/usr/bin/env python


import rospy
import cartographie_tl.msg
import geometry_msgs.msg
import std_msgs.msg


def convert(msg):
    leftWheelCommand=rospy.Publisher('vrep/leftWheelCommand',std_msgs.msg.Float64, queue_size=1)
    rightWheelCommand=rospy.Publisher('vrep/rightWheelCommand',std_msgs.msg.Float64, queue_size=1)
    rate=rospy.Rate(1)
    L=33*10**(-2)
    R=20*10**(-2)
    v=msg.linear.x
    w=msg.angular.z
    wl=(v-w*L/2)/R
    wr=(v+w*L/2)/R
    rospy.loginfo("I heard %s", wl)
    rospy.loginfo("I heard then %s", wr)

    leftWheelCommand.publish(std_msgs.msg.Float64(wl))
    rightWheelCommand.publish(std_msgs.msg.Float64(wr))

    rate.sleep()

    leftWheelCommand.publish(std_msgs.msg.Float64(0))
    rightWheelCommand.publish(std_msgs.msg.Float64(0))


def controler():
    rospy.init_node('controler', anonymous=True)
    hellow_world="hellow_world"
    rospy.loginfo(hellow_world)
    rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, convert)
    rospy.spin()



if __name__=='__main__':
    controler()


    dsllsklk
