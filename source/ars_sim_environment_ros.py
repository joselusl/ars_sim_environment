#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import visualization_msgs.msg
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker



import tf_conversions

import tf2_ros




#
import ars_lib_helpers





class ArsSimEnvironmentRos:

  #######


  # World frame
  world_frame = 'world'



  # Dynam obst loop freq 
  # time step
  dynamic_obst_loop_freq = 10.0
  # Timer
  dynamic_obst_loop_timer = None

  # Static obst loop freq 
  # time step
  static_obst_loop_freq = 10.0
  # Timer
  static_obst_loop_timer = None



  # Obstacles static pub
  obstacles_static_pub = None
  # Obstacles dynamic pub
  obstacles_dynamic_pub = None


  #
  obstacles_static_msg = None

  #
  obstacles_dynamic_msg = None

  


  #########

  def __init__(self):



    #
    self.obstacles_static_msg = MarkerArray()

    #
    self.obstacles_dynamic_msg = MarkerArray()


    # end
    return


  def init(self, node_name='ars_motion_controller_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_motion_controller')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###


    # Set static obstacles

    self.obstacles_static_msg = MarkerArray()

    self.obstacles_static_msg.markers = []


    # obstacle i

    obstacle_i = Marker()

    obstacle_i.header = Header()
    obstacle_i.header.stamp = rospy.Time()
    obstacle_i.header.frame_id = self.world_frame

    obstacle_i.ns = 'static'
    obstacle_i.id = 0

    obstacle_i.type = 3

    obstacle_i.action = 0

    obstacle_i.pose.position.x = 1.0
    obstacle_i.pose.position.y = 1.0
    obstacle_i.pose.position.z = 1.0

    obstacle_i.pose.orientation.w = 1.0
    obstacle_i.pose.orientation.x = 0.0
    obstacle_i.pose.orientation.y = 0.0
    obstacle_i.pose.orientation.z = 0.0

    obstacle_i.scale.x = 0.5
    obstacle_i.scale.y = 0.5
    obstacle_i.scale.z = 2.0

    obstacle_i.color.r = 1.0
    obstacle_i.color.g = 0.0
    obstacle_i.color.b = 0.0
    obstacle_i.color.a = 1.0

    obstacle_i.lifetime = rospy.Duration(0.0)

    self.obstacles_static_msg.markers.append(obstacle_i)

    #print(self.obstacles_static_msg)


    
    # End
    return


  def open(self):


    # Publishers

    # 
    self.obstacles_static_pub = rospy.Publisher('obstacles_static', MarkerArray, queue_size=1, latch=True)
    #
    self.obstacles_dynamic_pub = rospy.Publisher('obstacles_dynamic', MarkerArray, queue_size=1, latch=True)



    # Timers
    #
    self.dynamic_obst_loop_timer = rospy.Timer(rospy.Duration(1.0/self.dynamic_obst_loop_freq), self.dynamicObstaclesLoopTimerCallback)
    #
    self.static_obst_loop_timer = rospy.Timer(rospy.Duration(1.0), self.staticObstaclesLoopTimerCallback, oneshot=True)




    # End
    return


  def run(self):

    rospy.spin()

    return


  


  def updateDynObstaclesAndPublish(self, time_stamp_current):

    # Update
    # TODO


    # Publish dynamic obstacles
    self.obstacles_dynamic_pub.publish(self.obstacles_dynamic_msg)


    # End
    return


  def staticObstaclesLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #self.obstacles_static_msg = MarkerArray()

    #print(self.obstacles_static_msg)

    # Publish static obstacles
    self.obstacles_static_pub.publish(self.obstacles_static_msg)
    
    # End
    return

  

  def dynamicObstaclesLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    self.updateDynObstaclesAndPublish(time_stamp_current)

    
    # End
    return




  