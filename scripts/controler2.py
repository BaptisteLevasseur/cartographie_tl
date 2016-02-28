#!/usr/bin/env python


import rospy
import cartographie_tl.msg
import std_msgs.msg


def convert(msg):
    leftWheelCommand=rospy.Publisher('vrep/leftWheelCommand',std_msgs.msg.Float64, queue_size=1)
    rightWheelCommand=rospy.Publisher('vrep/rightWheelCommand',std_msgs.msg.Float64, queue_size=1)
    L=33*10**(-2)
    R=20*10**(-2)
    w=msg.w
    v=msg.v
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
    rospy.Subscriber('wheelCommand', cartographie_tl.msg.WheelCommand, convert)
    rospy.spin()



if __name__=='__main__':
    controler()


    dsllsklk
