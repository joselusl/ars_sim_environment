#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Bool
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


  #
  flag_dynamic_obstacles = True
  flag_dyn_obst_sub = None



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
    self.flag_dynamic_obstacles = True


    #
    self.obstacles_static_msg = MarkerArray()

    #
    self.obstacles_dynamic_msg = MarkerArray()


    # end
    return


  def init(self, node_name='ars_sim_environment_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_environment')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###


    # Obstacles static
    self.initObstaclesStatic()


    # Obstacles dynamic
    self.initObstaclesDynamic()


    
    # End
    return


  def open(self):


    # Publishers

    # 
    self.obstacles_static_pub = rospy.Publisher('obstacles_static', MarkerArray, queue_size=1, latch=True)
    #
    self.obstacles_dynamic_pub = rospy.Publisher('obstacles_dynamic', MarkerArray, queue_size=1, latch=True)


    # Subscribers
    self.flag_dyn_obst_sub = rospy.Subscriber('flag_dynamic_obstacles', Bool, self.flagDynamObstCallback)



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


  def flagDynamObstCallback(self, flag_dynamic_obstacles_msg):

    self.flag_dynamic_obstacles = flag_dynamic_obstacles_msg.data

    if(self.flag_dynamic_obstacles == True):
      self.initObstaclesDynamic()

    else:
      self.emptyObstaclesDynamic()

    # Publish
    self.publishObstacleDynamic()


    return


  def publishObstacleDynamic(self):

    self.obstacles_dynamic_pub.publish(self.obstacles_dynamic_msg)

    return

  
  def updateDynObstaclesAndPublish(self, time_stamp_current):

    #
    if(self.flag_dynamic_obstacles):

        # Update
        # TODO


        # Publish dynamic obstacles
        self.publishObstacleDynamic()


    # End
    return


  def staticObstaclesLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()


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


  def initObstaclesStatic(self):


    # Set static obstacles

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

    obstacle_i.pose.position.x = 3.0
    obstacle_i.pose.position.y = 3.0
    obstacle_i.pose.position.z = 1.0

    obstacle_i.pose.orientation.w = 1.0
    obstacle_i.pose.orientation.x = 0.0
    obstacle_i.pose.orientation.y = 0.0
    obstacle_i.pose.orientation.z = 0.0

    obstacle_i.scale.x = 2.0
    obstacle_i.scale.y = 2.0
    obstacle_i.scale.z = 2.0

    obstacle_i.color.r = 1.0
    obstacle_i.color.g = 0.0
    obstacle_i.color.b = 0.0
    obstacle_i.color.a = 0.3

    obstacle_i.lifetime = rospy.Duration(0.0)

    self.obstacles_static_msg.markers.append(obstacle_i)


    '''
    # obstacle i

    obstacle_i = Marker()

    obstacle_i.header = Header()
    obstacle_i.header.stamp = rospy.Time()
    obstacle_i.header.frame_id = self.world_frame

    obstacle_i.ns = 'static'
    obstacle_i.id = 0

    obstacle_i.type = 3

    obstacle_i.action = 0

    obstacle_i.pose.position.x = 2.0
    obstacle_i.pose.position.y = 2.0
    obstacle_i.pose.position.z = 1.0

    obstacle_i.pose.orientation.w = 1.0
    obstacle_i.pose.orientation.x = 0.0
    obstacle_i.pose.orientation.y = 0.0
    obstacle_i.pose.orientation.z = 0.0

    obstacle_i.scale.x = 1.5
    obstacle_i.scale.y = 1.5
    obstacle_i.scale.z = 2.0

    obstacle_i.color.r = 1.0
    obstacle_i.color.g = 0.0
    obstacle_i.color.b = 0.0
    obstacle_i.color.a = 0.3

    obstacle_i.lifetime = rospy.Duration(0.0)

    self.obstacles_static_msg.markers.append(obstacle_i)


    # obstacle i

    obstacle_i = Marker()

    obstacle_i.header = Header()
    obstacle_i.header.stamp = rospy.Time()
    obstacle_i.header.frame_id = self.world_frame

    obstacle_i.ns = 'static'
    obstacle_i.id = 2

    obstacle_i.type = 3

    obstacle_i.action = 0

    obstacle_i.pose.position.x = 4.0
    obstacle_i.pose.position.y = 4.0
    obstacle_i.pose.position.z = 1.0

    obstacle_i.pose.orientation.w = 1.0
    obstacle_i.pose.orientation.x = 0.0
    obstacle_i.pose.orientation.y = 0.0
    obstacle_i.pose.orientation.z = 0.0

    obstacle_i.scale.x = 1.5
    obstacle_i.scale.y = 1.5
    obstacle_i.scale.z = 2.0

    obstacle_i.color.r = 1.0
    obstacle_i.color.g = 0.0
    obstacle_i.color.b = 0.0
    obstacle_i.color.a = 0.3

    obstacle_i.lifetime = rospy.Duration(0.0)

    self.obstacles_static_msg.markers.append(obstacle_i)

    '''



    #
    return


  def removeObstaclesStatic(self):

    self.obstacles_static_msg.markers = []

    return


  def addObstaclesStatic(self):
    
    for obs_i_msg in self.obstacles_static_msg.markers:

      obs_i_msg.action = 0

    return


  def removeObstaclesStatic(self):

    for obs_i_msg in self.obstacles_static_msg.markers:

      obs_i_msg.action = 2

    return


  def initObstaclesDynamic(self):

    # Set dynamic obstacles

    self.obstacles_dynamic_msg.markers = []


    # obstacle i

    obstacle_i = Marker()

    obstacle_i.header = Header()
    obstacle_i.header.stamp = rospy.Time()
    obstacle_i.header.frame_id = self.world_frame

    obstacle_i.ns = 'dynamic'
    obstacle_i.id = 0

    obstacle_i.type = 3

    obstacle_i.action = 0

    obstacle_i.pose.position.x = 6.5
    obstacle_i.pose.position.y = 3.0
    obstacle_i.pose.position.z = 1.0

    obstacle_i.pose.orientation.w = 1.0
    obstacle_i.pose.orientation.x = 0.0
    obstacle_i.pose.orientation.y = 0.0
    obstacle_i.pose.orientation.z = 0.0

    obstacle_i.scale.x = 1.0
    obstacle_i.scale.y = 1.0
    obstacle_i.scale.z = 2.0

    obstacle_i.color.r = 0.0
    obstacle_i.color.g = 1.0
    obstacle_i.color.b = 0.0
    obstacle_i.color.a = 0.3

    obstacle_i.lifetime = rospy.Duration(1.0/self.dynamic_obst_loop_freq)

    self.obstacles_dynamic_msg.markers.append(obstacle_i)



    # obstacle i

    obstacle_i = Marker()

    obstacle_i.header = Header()
    obstacle_i.header.stamp = rospy.Time()
    obstacle_i.header.frame_id = self.world_frame

    obstacle_i.ns = 'dynamic'
    obstacle_i.id = 1

    obstacle_i.type = 3

    obstacle_i.action = 0

    obstacle_i.pose.position.x = 3.5
    obstacle_i.pose.position.y = 0.0
    obstacle_i.pose.position.z = 1.0

    obstacle_i.pose.orientation.w = 1.0
    obstacle_i.pose.orientation.x = 0.0
    obstacle_i.pose.orientation.y = 0.0
    obstacle_i.pose.orientation.z = 0.0

    obstacle_i.scale.x = 1.0
    obstacle_i.scale.y = 1.0
    obstacle_i.scale.z = 2.0

    obstacle_i.color.r = 0.0
    obstacle_i.color.g = 1.0
    obstacle_i.color.b = 0.0
    obstacle_i.color.a = 0.3

    obstacle_i.lifetime = rospy.Duration(1.0/self.dynamic_obst_loop_freq)

    self.obstacles_dynamic_msg.markers.append(obstacle_i)


    # obstacle i
    
    obstacle_i = Marker()

    obstacle_i.header = Header()
    obstacle_i.header.stamp = rospy.Time()
    obstacle_i.header.frame_id = self.world_frame

    obstacle_i.ns = 'dynamic'
    obstacle_i.id = 2

    obstacle_i.type = 3

    obstacle_i.action = 0

    obstacle_i.pose.position.x = 3.0
    obstacle_i.pose.position.y = 6.0
    obstacle_i.pose.position.z = 1.0

    obstacle_i.pose.orientation.w = 1.0
    obstacle_i.pose.orientation.x = 0.0
    obstacle_i.pose.orientation.y = 0.0
    obstacle_i.pose.orientation.z = 0.0

    obstacle_i.scale.x = 1.0
    obstacle_i.scale.y = 1.0
    obstacle_i.scale.z = 2.0

    obstacle_i.color.r = 0.0
    obstacle_i.color.g = 1.0
    obstacle_i.color.b = 0.0
    obstacle_i.color.a = 0.3

    obstacle_i.lifetime = rospy.Duration(1.0/self.dynamic_obst_loop_freq)

    self.obstacles_dynamic_msg.markers.append(obstacle_i)



    # obstacle i
    
    obstacle_i = Marker()

    obstacle_i.header = Header()
    obstacle_i.header.stamp = rospy.Time()
    obstacle_i.header.frame_id = self.world_frame

    obstacle_i.ns = 'dynamic'
    obstacle_i.id = 3

    obstacle_i.type = 3

    obstacle_i.action = 0

    obstacle_i.pose.position.x = 0.5
    obstacle_i.pose.position.y = 3.0
    obstacle_i.pose.position.z = 1.0

    obstacle_i.pose.orientation.w = 1.0
    obstacle_i.pose.orientation.x = 0.0
    obstacle_i.pose.orientation.y = 0.0
    obstacle_i.pose.orientation.z = 0.0

    obstacle_i.scale.x = 1.0
    obstacle_i.scale.y = 1.0
    obstacle_i.scale.z = 2.0

    obstacle_i.color.r = 0.0
    obstacle_i.color.g = 1.0
    obstacle_i.color.b = 0.0
    obstacle_i.color.a = 0.3

    obstacle_i.lifetime = rospy.Duration(1.0/self.dynamic_obst_loop_freq)

    self.obstacles_dynamic_msg.markers.append(obstacle_i)
    


    return


  def emptyObstaclesDynamic(self):

    self.obstacles_dynamic_msg.markers = []

    return
