#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from drone_interfaces.msg import CtlTraj
from rviz_drone.config import SCALE_SIM


class TrajViz(Node):
    def __init__(self):
        super().__init__("traj_visualizer")

        self.declare_parameter('scale_size', 0.05)
        self.declare_parameter('life_time', 2.0)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'trajectory_frame')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('ns', 'trajectory')
        self.declare_parameter('topic_name', 'trajectory')
        self.declare_parameter('red_color', 1.0)
        self.declare_parameter('green_color', 0.0)
        self.declare_parameter('blue_color', 0.0)
        self.declare_parameter('alpha_color', 1.0)
        
        self.scale_size = self.get_parameter('scale_size').value
        self.life_time = self.get_parameter('life_time').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.rate = int(self.get_parameter('rate').value)
        self.ns = self.get_parameter('ns').get_parameter_value().string_value
        self.topic_name = str(self.get_parameter('topic_name').get_parameter_value().string_value)
        
        self.red_color = self.get_parameter('red_color').value
        self.green_color = self.get_parameter('green_color').value
        self.blue_color = self.get_parameter('blue_color').value
        self.alpha_color = self.get_parameter('alpha_color').value
        
        self.traj_sub = self.create_subscription(CtlTraj, '/trajectory',
                                                self.trajCallback, self.rate)
        
        self.traj_pub = self.create_publisher(MarkerArray, 
                                            self.topic_name+"_marker", 
                                              self.rate)
        
        self.traj_counter = 0
        self.marker_array = MarkerArray()
        self.max_markers = 100
        
    def trajCallback(self,msg:CtlTraj)->None:
        """
        Visualize trajectory waypoints
        """
        scale_size = self.scale_size

        # keep in the trajectories are in NED frame, 
        # need to rotate this to ENU frame
        marker_array = self.marker_array
        
        x_waypoints = msg.y
        y_waypoints = msg.x
        z_waypoints = msg.z         

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

        if self.traj_counter > self.max_markers:
            marker_array.markers.pop(0)
            
        id_num = 0
        for x_wp,y_wp,z_wp in zip(x_waypoints,y_waypoints,z_waypoints):

            wp = Point()
            wp.x = x_wp/SCALE_SIM
            wp.y = y_wp/SCALE_SIM
            wp.z = (-z_wp)/SCALE_SIM
            
            marker.points.append(wp)

            marker.id = id_num
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "/map"
            marker.ns = self.ns
            # marker.namespace = "trajectory"
            marker.lifetime = rclpy.duration.Duration(seconds=self.life_time).to_msg()
            marker_array.markers.append(marker)
            
            id_num += 1
            self.traj_counter += 1
            
        print(f'Publishing {len(marker_array.markers)} markers')

        self.traj_pub.publish(marker_array)

def main()->None:
    rclpy.init()
    node = TrajViz()

    while rclpy.ok():
        rclpy.spin(node)

    rclpy.shutdown()

if __name__=="__main__":
    main()    