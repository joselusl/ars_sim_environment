#!/usr/bin/env python3

import rclpy

from ars_sim_environment.ars_sim_environment_ros import ArsSimEnvironmentRos


def main(args=None):

  rclpy.init(args=args)

  ars_sim_environment_ros = ArsSimEnvironmentRos()

  ars_sim_environment_ros.open()

  try:
      ars_sim_environment_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_sim_environment_ros.destroy_node()
    rclpy.try_shutdown()

  return 0



''' MAIN '''
if __name__ == '__main__':

  main()