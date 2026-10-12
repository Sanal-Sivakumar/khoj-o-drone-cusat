#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from waypoint_navigation.srv import GetWaypoints

class WayPoints(Node):

    def __init__(self):
        super().__init__('waypoints_service')
        self.srv = self.create_service(GetWaypoints, 'waypoints', self.waypoint_callback)
        
        # Predefined Task 2B waypoints in strict official order: wp1, wp2, wp3, wp4
        self.waypoints = [
            [-2.41, -2.41, 13.00],
            [2.41, 1.20, 13.00],
            [1.20, -2.41, 13.00],
            [-1.20, 1.20, 13.00]
        ]
        self.marker_ids = ['wp1', 'wp2', 'wp3', 'wp4']
        self.get_logger().info('WayPoints service initialized with 4 waypoints.')

    def waypoint_callback(self, request, response):
        if request.get_waypoints:
            response.waypoints.poses = [Pose() for _ in range(len(self.waypoints))]
            for i in range(len(self.waypoints)):
                response.waypoints.poses[i].position.x = float(self.waypoints[i][0])
                response.waypoints.poses[i].position.y = float(self.waypoints[i][1])
                response.waypoints.poses[i].position.z = float(self.waypoints[i][2])
            response.marker_ids = self.marker_ids
            self.get_logger().info('Incoming request for Waypoints: successfully served 4 waypoints.')
            return response
        else:
            self.get_logger().warn('Incoming request with get_waypoints=False: request rejected.')
            return response

def main():
    rclpy.init()
    waypoints = WayPoints()

    try:
        rclpy.spin(waypoints)
    except KeyboardInterrupt:
        waypoints.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoints.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        