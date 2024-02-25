#!/usr/bin/env python3

import rclpy
#import math
#import numpy as np


from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from drone_interfaces.msg import CtlTraj
from rviz_drone.config import SCALE_SIM

# import mavros
# from mavros.base import SENSOR_QOS


class TrajViz(Node):
    def __init__(self):
        super().__init__("traj_visualizer")

        self.declare_parameter('scale_size', 0.25)
        self.declare_parameter('life_time', 2.0)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'trajectory_frame')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('ns', 'trajectory')
        self.declare_parameter('topic_name', 'trajectory')
        self.declare_parameter('scale_size', 0.05)
        self.declare_parameter('red_color', 1.0)
        self.declare_parameter('green_color', 0.0)
        self.declare_parameter('blue_color', 0.0)
        self.declare_parameter('alpha_color', 0.5)
        
        self.scale_size = self.get_parameter('scale_size').value
        self.life_time = self.get_parameter('life_time').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.rate = int(self.get_parameter('rate').value)
        self.ns = self.get_parameter('ns').get_parameter_value().string_value
        self.topic_name = str(self.get_parameter('topic_name').get_parameter_value().string_value)
        print(self.topic_name)
        self.scale_size = self.get_parameter('scale_size').value
        self.red_color = self.get_parameter('red_color').value
        self.green_color = self.get_parameter('green_color').value
        self.blue_color = self.get_parameter('blue_color').value
        self.alpha_color = self.get_parameter('alpha_color').value
        
        # TODO: Add a parameter for the topic name
        self.traj_sub = self.create_subscription(CtlTraj, self.topic_name,
                                                self.trajCallback, self.rate)
        
        self.traj_counter = 0

        # self.position_frame = 
        
        # self.marker_pub = self.create_publisher(MarkerArray,    
        #                                         "waypoint_marker", 
        #                                         10)
        
        self.traj_pub = self.create_publisher(MarkerArray, 
                                            self.topic_name+"_marker", 
                                              self.rate)
        
        # self.obs_pub = self.create_publisher(MarkerArray,
        #                                     "obstacle_marker",
        #                                     10)

        # self.wp_sub = self.create_subscription(Waypoints, "/global_waypoints", 
        #                                        self.waypointCallback, 10)
                
        # self.mpc_wp_sub = self.create_subscription(CtlTraj, "trajectory",
        #                                             self.mpcCallback, 10)
        
        # self.obs_pos_sub = self.create_subscription(Waypoints, "/obs_positions",
        #                                             self.obsPosCallback, 10)

        # self.counter = 0
        # self.traj_counter = 0
        # self.id = 0
        
    def trajCallback(self,msg:CtlTraj)->None:
        """
        Visualize trajectory waypoints
        """
        scale_size = self.scale_size

        # keep in the trajectories are in NED frame, 
        # need to rotate this to ENU frame
        x_waypoints = msg.y
        y_waypoints = msg.x
        z_waypoints = msg.z         
        marker_array = MarkerArray()

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = scale_size
        marker.scale.y = scale_size
        marker.scale.z = scale_size

        marker.color.r = self.red_color
        marker.color.b = self.blue_color
        marker.color.g = self.green_color
        marker.color.a = self.alpha_color

        for x_wp,y_wp,z_wp in zip(x_waypoints,y_waypoints,z_waypoints):

            wp = Point()
            wp.x = x_wp/SCALE_SIM
            wp.y = y_wp/SCALE_SIM
            wp.z = (-z_wp)/SCALE_SIM
            
            marker.points.append(wp)

            marker.id = self.traj_counter
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "/map"
            marker.ns = self.ns
            # marker.namespace = "trajectory"
            marker.lifetime = rclpy.duration.Duration(seconds=self.life_time).to_msg()

            marker_array.markers.append(marker)
            self.traj_counter += 1
        print("Publishing trajectory")
        self.traj_pub.publish(marker_array)

    # def waypointCallback(self,msg:Waypoints)->None:
    #     """
    #     Listen for waypoints and publish waypoints
    #     """
    #     scale_size = 0.25
    #     waypoints = msg.points        
    #     marker_array = MarkerArray()

    #     marker = Marker()
    #     marker.type = Marker.SPHERE_LIST
    #     marker.action = Marker.ADD
        
    #     marker.scale.x = scale_size
    #     marker.scale.y = scale_size
    #     marker.scale.z = scale_size

    #     marker.color.b = 1.0
    #     marker.color.a = 1.0    

    #     for wp in waypoints:

    #         wp.x = wp.x/SCALE_SIM
    #         wp.y = wp.y/SCALE_SIM
    #         wp.z = -wp.z/SCALE_SIM
            
    #         marker.points.append(wp)

    #         marker.id = self.counter
    #         marker.header.stamp = self.get_clock().now().to_msg()
    #         marker.header.frame_id = "/map"
    #         marker.lifetime = rclpy.duration.Duration(seconds=2).to_msg()

    #         marker_array.markers.append(marker)
    #         self.counter += 1

    #     self.marker_pub.publish(marker_array)

    # def mpcCallback(self,msg:CtlTraj)->None:
    #     """
    #     Visualize trajectory waypoints
    #     """
    #     scale_size = 0.1

    #     # keep in the trajectories are in NED frame, 
    #     # need to rotate this to ENU frame
    #     x_waypoints = msg.y
    #     y_waypoints = msg.x
    #     z_waypoints = msg.z         
    #     marker_array = MarkerArray()

    #     marker = Marker()
    #     marker.type = Marker.LINE_STRIP
    #     marker.action = Marker.ADD
        
    #     marker.scale.x = scale_size
    #     marker.scale.y = scale_size
    #     marker.scale.z = scale_size

    #     marker.color.g = 1.0
    #     marker.color.a = 1.0    

    #     for x_wp,y_wp,z_wp in zip(x_waypoints,y_waypoints,z_waypoints):

    #         wp = Point()
    #         wp.x = x_wp/Config.SCALE_SIM
    #         wp.y = y_wp/Config.SCALE_SIM
    #         wp.z = (-z_wp/Config.SCALE_SIM)
            
    #         marker.points.append(wp)

    #         marker.id = self.traj_counter
    #         marker.header.stamp = self.get_clock().now().to_msg()
    #         marker.header.frame_id = "/map"
    #         marker.ns = "trajectory"
    #         # marker.namespace = "trajectory"
    #         marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()

    #         marker_array.markers.append(marker)
    #         self.traj_counter += 1

    #     self.traj_pub.publish(marker_array)

    # def obsPosCallback(self,msg:Waypoints)->None:
    #     """
    #     Listen for waypoints and publish waypoints
    #     """
    #     scale_size = 0.25
    #     waypoints = msg.points        
    #     marker_array = MarkerArray()

    #     marker = Marker()
    #     marker.type = Marker.SPHERE_LIST
    #     marker.action = Marker.ADD
        
    #     marker.scale.x = scale_size
    #     marker.scale.y = scale_size
    #     marker.scale.z = scale_size

    #     marker.color.r = 1.0
    #     marker.color.a = 1.0    

    #     for wp in waypoints:

    #         wp.x = wp.x/Config.SCALE_SIM
    #         wp.y = wp.y/Config.SCALE_SIM
    #         wp.z = wp.z/Config.SCALE_SIM
            
    #         marker.points.append(wp)

    #         marker.id = self.counter
    #         marker.header.stamp = self.get_clock().now().to_msg()
    #         marker.header.frame_id = "/map"
    #         marker.ns = "obstacles"

    #         marker_array.markers.append(marker)
    #         self.counter += 1

    #     self.obs_pub.publish(marker_array)

def main()->None:
    rclpy.init()
    node = TrajViz()

    #node.publishWaypointMarkers()
    while rclpy.ok():
        # node.cool_example()
        rclpy.spin(node)
        # rclpy.spin_once(node, timeout_sec=0.1)

    rclpy.shutdown()

if __name__=="__main__":
    main()    