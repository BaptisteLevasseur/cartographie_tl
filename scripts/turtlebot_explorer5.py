#!/usr/bin/env python
# coding: utf-8

# This node communicates with the actionlib server of move_base
# by sending it goals as a Pose in the /map frame

# http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20(Python)

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.srv import GetMap
import tf
from math import pi, cos, sin


import scipy.misc
import numpy as np


rospy.init_node('explore_client')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

listener = tf.TransformListener()

###### Some global defintions

goal_number=0
target_frame='map'
base_link='base_link'


def reach_goal(x, y, theta):
  target_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

  t0=rospy.Time.now()
  goal=MoveBaseGoal()
  goal.target_pose.header.seq=goal_number
  goal.target_pose.header.stamp=t0
  goal.target_pose.header.frame_id=target_frame

  goal.target_pose.pose.position = Point(x, y, 0)
  goal.target_pose.pose.orientation.x = target_quat[0]
  goal.target_pose.pose.orientation.y = target_quat[1]
  goal.target_pose.pose.orientation.z = target_quat[2]
  goal.target_pose.pose.orientation.w = target_quat[3]
  print(str(goal))

  # Sends the goal to the action server.
  client.send_goal(goal)

  # Waits for the server to finish performing the action.
  client.wait_for_result()
  print(client.get_result())

  # Prints out the result of executing the action
  return client.get_result()



# First goal : turn around to scan the surroundings
#t0 = rospy.Time(0)
#listener.waitForTransform('map', base_link, t0, rospy.Duration(1))
#((x,y,z), rot) = listener.lookupTransform('map', base_link, t0)
#euler = tf.transformations.euler_from_quaternion(rot)
#reach_goal(x, y, euler[2] + pi)
#print("G0 done")

#t0 = rospy.Time(0)
#listener.waitForTransform('map', base_link, t0, rospy.Duration(1))
#((x,y,z), rot) = listener.lookupTransform('map', base_link, t0)
#euler = tf.transformations.euler_from_quaternion(rot)
#reach_goal(x, y, euler[2] + pi)
#print("G1 done")


###### We now look for places to go....
# The map is accessed by calling the service dynamic_map()
get_map = rospy.ServiceProxy('dynamic_map', GetMap)


# Some useful functions for converting a position in the world in the
# map frame into its coordinates in the "map", i.e. in the image

def pose_to_pix(pose_robot, pose_origin, metadata):
  w = metadata[0]
  h = metadata[1]
  res = metadata[2]
  # We first convert pose_robot from the "map" frame to the image frame
  x_robot, y_robot,theta_robot = pose_robot
  x_origin, y_origin, theta_origin= pose_origin
  tempx=x_origin+x_robot
  tempy=y_origin+x_robot
  xr_in_im = (y_origin - y_robot) / res + h
  yr_in_im = (x_robot -  x_origin) / res  
#  ###### For the position
#  # The translation and scaling
#  xr_in_im = (x_robot - x_origin)/res * cos(-theta_origin)-(y_robot - y_origin)/res * sin(-theta_origin)
#  yr_in_im = (y_robot - y_origin)/res * sin(-theta_origin)+(y_robot - y_origin)/res * cos(-theta_origin)
#  # And apply a rotation
  theta_in_im = theta_robot - theta_origin
  return (int(xr_in_im), int(yr_in_im), theta_in_im)

def pix_to_pose(pose_robot_in_im, pose_origin, metadata):
  w = metadata[0]
  h = metadata[1]
  res = metadata[2]
  # We first convert pose_robot from the "map" frame to the image frame
  x_robot_in_im, y_robot_in_im,theta_robot_in_im = pose_robot_in_im
  x_origin, y_origin, theta_origin= pose_origin

  x_robot= y_robot_in_im * res  + x_origin
  y_robot= -((x_robot_in_im - h) * res - y_origin)
  theta_robot = theta_robot_in_im + theta_origin
  return (x_robot, y_robot, theta_robot)


# We now start the controller and should in principle run the node
# until no reachable locations remain unknown 
# (i.e. there will be no unknown places next to a known free place)

# Request the map as well as the metadata
m = get_map().map
width=m.info.width
height = m.info.height
res = m.info.resolution
origin = m.info.origin
data = m.data
metadata = (width, height, res)
x_origin = origin.position.x
y_origin = origin.position.y
theta_origin =  (tf.transformations.euler_from_quaternion((origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w)))[2]
pose_origin = (x_origin, y_origin, theta_origin)




# we also need to know where we are
t0 = rospy.Time(0)
listener.waitForTransform('map', base_link, t0, rospy.Duration(1))
((x_robot,y_robot,z), rot) = listener.lookupTransform('map', base_link, t0)
euler = tf.transformations.euler_from_quaternion(rot)
theta_robot = euler[2]
pose_robot = (x_robot, y_robot, theta_robot)


# The robot is at (x_robot, y_robot, theta_robot) in the "map" frame
# The origin is at (x_origin, y_origin, theta_origin) in the "map" frame

# DEMO : we demonstrate how to use pose_to_pix and pix_to_pose
print("Original pose in map : " + str(pose_robot))
pose_in_im = pose_to_pix((x_robot, y_robot, theta_robot), pose_origin, metadata)
print("Pose in the image : " + str(pose_in_im))
pose_in_map = pix_to_pose(pose_in_im, pose_origin, metadata)
print("From which we compute the pose in map :" + str(pose_in_map))


# DEMO : we generate an image where we plot the map
# as well as the location and orientation of the robot
image_array = np.zeros((height, width,3), dtype=int)


copyData=list(data)
for i in range(height):
  for j in range(width):
      copyData[i*width+j]=data[(height-1-i)*width+j]

# Plotting the map
for i in range(height):
  for j in range(width):
    if(copyData[i*width+j] == -1): # Unknown
      image_array[i,j,0] = 255
      image_array[i,j,1] = 255
      image_array[i,j,2] = 255
    elif(copyData[i*width+j] == 0): # Free
      image_array[i,j,0] = 125
      image_array[i,j,1] = 125
      image_array[i,j,2] = 125
    elif(copyData[i*width+j] == 100): # Walls
      image_array[i,j,0] = 0
      image_array[i,j,1] = 0
      image_array[i,j,2] = 0

# Retourne "True" si le pixel (x,y) est adjacent à une bordure et est dans la zone accessible
def is_free(x,y):
    free=True
    rayon_inflate=7
    if(copyData[x*width+y]==-2):
        for k in range(-1,2):
            for l in range(-1,2): # On regarde les cellules adjacentes
                if(k != 0 or l != 0):
                    if(copyData[(x+k)*width+(y+l)] == -1):  # Si une de ces cellules adjacentes est inconnue
                        print("Bordure trouvée")
                        image_array[x,y,0] = 255
                        image_array[x,y,1] = 20
                        image_array[x,y,2] = 147
                        return True
    else:
        return False


# /!\ Il faudra étudier s'il est vraiment nécessaire de recréer un vecteur "copyData" => Apparemment on ne peut pas modifier data avec des valeurs interdites.
# Retourne "True" si le pixel (x,y) est accessible par le robot (situé à un rayon donné des murs)
def is_accessible(x,y): #Renvoie True si l'on n'est pas près d'un mur
    rayon_inflate=7
    if(copyData[x*width+y]==0):
        for m in range(-rayon_inflate,rayon_inflate+1): #On regarde si l'on n'est pas près d'un mur (costmap)
            for n in range(-rayon_inflate,rayon_inflate+1):
                if(copyData[(x+m)*width+(y+n)]==100):
                    return False
        return True
    return False

def remplissage_diff(): #Diffuse la zone d'accessibilité en prenant en compte l'espacement des murs
    pile=[]
    x_robot=pose_in_im[0] #Position du robot
    y_robot=pose_in_im[1]
    if not is_accessible(x_robot,y_robot): #Si il y un mur autour de ce pixel
        return
    pile.append([x_robot,y_robot])
    print("Analyse des zones accessibles")
    while pile != []:
        [x,y]=pile.pop()
        image_array[x,y,0] = 30
        image_array[x,y,1] =250
        image_array[x,y,2] = 10
        copyData[x*width+y]=-2 #La valeur arbitraire "-2" correspond à une zone accessible pour le robot
        for k in range(-1,2):
            for l in range(-1,2): #Pour chaque pixels adjacents au pixel actuel, on regarde si la zone est accessible
                if(k !=0 or l !=0):
                    if is_accessible(x+k,y+l):
                        pile.append([x+k,y+l])
    print("Analyse terminée")
    return


def find_ppv(): #Cherche le plus proche voisin libre 
    rayon=3
    x_robot=pose_in_im[0]
    y_robot=pose_in_im[1]
    print("Recherche du plus proche voisin")
    while abs(rayon+x_robot)< width and abs(rayon+y_robot) < height:
        for i in range(-rayon,rayon+1): #Parcourt la carte en partant de la position initiale du robot en faisant des carr
            if(i == -rayon or i==rayon): #Si on est sur un bord de gauche ou de droite
                for j in range(-rayon,rayon+1):
                    if(is_free(x_robot+i,y_robot+j)):
                        return (x_robot+i,y_robot+j)
            else:
                if(is_free(x_robot+i,y_robot+rayon)):
                    return (x_robot+i,y_robot+rayon)
                if(is_free(x_robot+i,y_robot-rayon)):
                    return (x_robot+i,y_robot-rayon)
	rayon = rayon+1



remplissage_diff()
image_array[200,0]=(50,50,50)
scipy.misc.imsave('map.png', image_array)
(x_im,y_im)=find_ppv()
(x,y,theta)=pix_to_pose((x_im,y_im,0), pose_origin, metadata)
print(x,y,theta)



# Plotting the location of the robot
for i in range(-3,4):
  for j in range(-3,4):
    image_array[pose_in_im[0]+i, pose_in_im[1]+j] = (255, 0, 0)
# Plotting its orientation
for i in range(10):
  image_array[int(pose_in_im[0]+i*sin(-pose_in_im[2])), int(pose_in_im[1]+i*cos(-pose_in_im[2]))] = (0, 0, 255)


print("Enregistrement de l'image")

reach_goal(x,y,theta) 
