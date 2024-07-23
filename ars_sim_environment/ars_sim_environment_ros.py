import numpy as np
from numpy import *

import os

# pyyaml - https://pyyaml.org/wiki/PyYAMLDocumentation
import yaml
from yaml.loader import SafeLoader



# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

#import
from ament_index_python.packages import get_package_share_directory

import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers




class ArsSimEnvironmentRos(Node):

  #######

  # World frame
  world_frame = None

  #
  environment_descript = None
  environment_descript_yaml_file_name = None

  #
  flag_dynamic_obstacles = None
  flag_dyn_obst_sub = None

  # Dynam obst loop freq 
  # time step
  dynamic_obst_loop_freq = None
  # Timer
  dynamic_obst_loop_timer = None

  # Static obst loop freq 
  # time step
  static_obst_loop_freq = None
  # Timer
  static_obst_loop_timer = None
  static_obst_loop_timer_active = False 


  # Obstacles static pub
  obstacles_static_pub = None
  # Obstacles dynamic pub
  obstacles_dynamic_pub = None

  #
  obstacles_static_msg = None
  #
  obstacles_dynamic_msg = None

  #
  id_first_available = None
  

  #########

  def __init__(self, node_name='ars_sim_environment_node'):

    # Init ROS
    super().__init__(node_name)

    # World frame
    self.world_frame = 'world'

    #
    self.environment_descript = None
    self.environment_descript_yaml_file_name = ''

    #
    self.flag_dynamic_obstacles = True
    self.flag_dyn_obst_sub = None

    # Dynam obst loop freq 
    # time step
    self.dynamic_obst_loop_freq = 10.0
    # Timer
    self.dynamic_obst_loop_timer = None

    # Static obst loop freq 
    # time step
    self.static_obst_loop_freq = 10.0
    # Timer
    self.static_obst_loop_timer = None
    self.static_obst_loop_timer_active = False 
    
    #
    self.obstacles_static_msg = MarkerArray()
    #
    self.obstacles_dynamic_msg = MarkerArray()

    #
    self.id_first_available = 0

    self.__init(node_name)

    # end
    return


  def __init(self, node_name='ars_sim_environment_node'):
    
    # Package path
    try:
      pkg_path = get_package_share_directory('ars_sim_environment')
      print(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      print("Package not found")
    

    #### READING PARAMETERS ###
    
    # Environment description
    default_environment_descript_yaml_file_name = os.path.join(pkg_path,'config','obstacles_env_01.yaml')
    # Declare the parameter with a default value
    self.declare_parameter('environment_description_yaml_file', default_environment_descript_yaml_file_name)
    # Get the parameter value
    environment_descript_yaml_file_name_str = self.get_parameter('environment_description_yaml_file').get_parameter_value().string_value
    print(environment_descript_yaml_file_name_str)
    #
    self.environment_descript_yaml_file_name = os.path.abspath(environment_descript_yaml_file_name_str)


    ###

    # Load environment description
    with open(self.environment_descript_yaml_file_name,'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        self.environment_descript = yaml.load(file, Loader=SafeLoader)



    # Obstacles static
    self.initObstaclesStatic()

    # Obstacles dynamic
    self.initObstaclesDynamic()

    
    # End
    return


  def open(self):

    # Publishers

    # 
    obstacles_static_qos_profile = rclpy.qos.QoSProfile(depth=1)
    obstacles_static_qos_profile.history=rclpy.qos.HistoryPolicy.KEEP_LAST
    obstacles_static_qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    obstacles_static_qos_profile.reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
    self.obstacles_static_pub = self.create_publisher(MarkerArray, 'obstacles_static', obstacles_static_qos_profile)
    #
    self.obstacles_dynamic_pub = self.create_publisher(MarkerArray, 'obstacles_dynamic', qos_profile=10)

    # Subscribers
    self.flag_dyn_obst_sub = self.create_subscription(Bool, 'flag_dynamic_obstacles', self.flagDynamObstCallback, qos_profile=10)


    # Timers
    #
    self.dynamic_obst_loop_timer = self.create_timer(1.0/self.dynamic_obst_loop_freq, self.dynamicObstaclesLoopTimerCallback)
    #
    self.static_obst_loop_timer = self.create_timer(1.0, self.staticObstaclesLoopTimerCallback)
    self.static_obst_loop_timer_active = True


    # End
    return


  def run(self):

    rclpy.spin(self)

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


  def staticObstaclesLoopTimerCallback(self):

    if self.static_obst_loop_timer_active:

      # Get time
      time_stamp_current = self.get_clock().now()

      # Publish static obstacles
      self.obstacles_static_pub.publish(self.obstacles_static_msg)

      # Deactivate the timer
      self.static_obst_loop_timer_active = False
      self.static_obst_loop_timer.cancel()
    
    # End
    return


  def dynamicObstaclesLoopTimerCallback(self):

    # Get time
    time_stamp_current = self.get_clock().now()

    #
    self.updateDynObstaclesAndPublish(time_stamp_current)

    
    # End
    return


  def initObstaclesStatic(self):


    # Set static obstacles

    self.obstacles_static_msg.markers = []

    #
    print('Obstacles static:')

    for object_env in self.environment_descript['static']:

        print('Circles:')

        for circle in object_env['circles']:

            #
            print(circle)


            # obstacle i

            obstacle_i = Marker()

            obstacle_i.header = Header()
            obstacle_i.header.stamp = self.get_clock().now().to_msg()
            obstacle_i.header.frame_id = self.world_frame

            obstacle_i.ns = 'static'
            obstacle_i.id = self.id_first_available

            obstacle_i.action = 0

            obstacle_i.type = 3

            obstacle_i.pose.position.x = circle['position'][0]
            obstacle_i.pose.position.y = circle['position'][1]
            obstacle_i.pose.position.z = circle['position'][2]

            obstacle_i.pose.orientation.w = 1.0
            obstacle_i.pose.orientation.x = 0.0
            obstacle_i.pose.orientation.y = 0.0
            obstacle_i.pose.orientation.z = 0.0

            obstacle_i.scale.x = circle['sizes'][0]
            obstacle_i.scale.y = circle['sizes'][1]
            obstacle_i.scale.z = circle['sizes'][2]

            obstacle_i.color.r = 1.0
            obstacle_i.color.g = 0.0
            obstacle_i.color.b = 0.0
            obstacle_i.color.a = 0.3

            obstacle_i.lifetime = Duration(seconds=0.0).to_msg()

            #
            self.obstacles_static_msg.markers.append(obstacle_i)

            #
            self.id_first_available += 1


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


    #
    print('Obstacles dynamic:')

    for object_env in self.environment_descript['dynamic']:

        print('Circles:')

        for circle in object_env['circles']:

            #
            print(circle)


            # obstacle i

            obstacle_i = Marker()

            obstacle_i.header = Header()
            obstacle_i.header.stamp = self.get_clock().now().to_msg()
            obstacle_i.header.frame_id = self.world_frame

            obstacle_i.ns = 'dynamic'
            obstacle_i.id = self.id_first_available

            obstacle_i.action = 0

            obstacle_i.type = 3

            obstacle_i.pose.position.x = circle['position'][0]
            obstacle_i.pose.position.y = circle['position'][1]
            obstacle_i.pose.position.z = circle['position'][2]

            obstacle_i.pose.orientation.w = 1.0
            obstacle_i.pose.orientation.x = 0.0
            obstacle_i.pose.orientation.y = 0.0
            obstacle_i.pose.orientation.z = 0.0

            obstacle_i.scale.x = circle['sizes'][0]
            obstacle_i.scale.y = circle['sizes'][1]
            obstacle_i.scale.z = circle['sizes'][2]

            obstacle_i.color.r = 0.0
            obstacle_i.color.g = 1.0
            obstacle_i.color.b = 0.0
            obstacle_i.color.a = 0.3

            obstacle_i.lifetime = Duration(seconds=1.0/self.dynamic_obst_loop_freq).to_msg()

            #
            self.obstacles_dynamic_msg.markers.append(obstacle_i)

            #
            self.id_first_available += 1


    return


  def emptyObstaclesDynamic(self):

    self.obstacles_dynamic_msg.markers = []

    return
