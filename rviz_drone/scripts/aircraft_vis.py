#!/usr/bin/env python3

import rclpy
import math
import numpy as np


from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point
from drone_interfaces.msg import Telem, CtlTraj

from visualization_msgs.msg import Marker, MarkerArray
from drone_interfaces.msg import Waypoints,CtlTraj

from rviz_drone.config import SCALE_SIM

import mavros
from mavros.base import SENSOR_QOS

"""
Visualizes the direction frame of the aircraft
- Subscribe to the position of aircraft and visualize the direction frame 
- Publish as a frame in RVIZ


Refer to documentation given:
https://github.com/jrgnicho/ros2_support_utilities/blob/master/rviz2_py/rviz2_py/simple_rviz_app.py

http://wiki.ros.org/rviz/DisplayTypes/Marker

"""

class AircraftFrameViz(Node):
	def __init__(self, 
				 pub_freq:int=30, 
				 sub_freq:int=30,
				 sub_to_mavros:bool=True,
	 			 life_time:float=2.0):
		super().__init__('aircraft_frame_viz')
		self.get_logger().info('Starting Aircraft Frame Viz Node')

		self.pub_freq = pub_freq
		self.sub_freq = sub_freq
		self.sub_to_mavros = sub_to_mavros
		self.life_time = life_time

		if sub_to_mavros:
			self.state_sub = self.create_subscription(mavros.local_position.Odometry,
													'mavros/local_position/odom', 
													self.mavros_state_callback, 
													qos_profile=SENSOR_QOS)
		else:        
			self.state_sub = self.create_subscription(Telem, 
				'telem',
				self.state_callback,
				self.sub_freq)
	
		self.state_info = [
			None, #x
			None, #y
			None, #z
			None, #phi
			None, #theta
			None, #psi
			None, #airspeed
		]
 	
		self.forward_arrow_pub = self.create_publisher(Marker,
												 "forward_aircraft",
												 self.pub_freq)
												 

	
	def mavros_state_callback(self, msg:mavros.local_position.Odometry)->None:
		# print(msg)
		self.state_info[0] = msg.pose.pose.position.x
		self.state_info[1] = msg.pose.pose.position.y
		self.state_info[2] = msg.pose.pose.position.z
		self.state_info[3] = msg.pose.pose.orientation.x
		self.state_info[4] = msg.pose.pose.orientation.y
		self.state_info[5] = msg.pose.pose.orientation.z
		self.state_info[6] = msg.twist.twist.linear.x
  
		marker = Marker()
		marker.header.stamp = self.get_clock().now().to_msg()
		marker.header.frame_id = "map"
		marker.lifetime = rclpy.duration.Duration(seconds=self.life_time).to_msg()

		start_point = Point()
		#create a point at the current position
		start_point.x = self.state_info[0]/SCALE_SIM
		start_point.y = self.state_info[1]/SCALE_SIM
		start_point.z = self.state_info[2]/SCALE_SIM
  
		end_point = Point()
		#find the unit vector in the direction of the aircraft
		#and scale it by the airspeed
		end_point.x = (self.state_info[6] * math.cos(self.state_info[5]) * \
      		math.cos(self.state_info[4]))/SCALE_SIM
  
		end_point.y = (self.state_info[6] * math.sin(self.state_info[5]) * \
			math.cos(self.state_info[4]))/SCALE_SIM

		end_point.z = (self.state_info[6] * math.sin(self.state_info[4]))/SCALE_SIM 
  
		marker.points.append(start_point)
		marker.points.append(end_point)
  
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.w = 1.0
  
		marker.id = 0
		marker.color.r = 1.0 #red
		marker.color.g = 0.0 #green
		marker.color.b = 0.0 #blue
		marker.color.a = 0.5 #transparency
  
		marker.scale.x = 0.2
		marker.scale.y = 1.0
		marker.scale.z = 1.0

		self.forward_arrow_pub.publish(marker)
  
		# marker.type = Marker.ARROW
		# marker.action = Marker.ADD

		
 
	def state_callback(self, msg:Telem)->None:
		# print(msg)
		self.state_info[0] = msg.x
		self.state_info[1] = msg.y
		self.state_info[2] = msg.z
		self.state_info[3] = msg.phi
		self.state_info[4] = msg.theta
		self.state_info[5] = msg.psi
		self.state_info[6] = msg.airspeed
	

def main()->None:
	rclpy.init()
	aircraft_vis_node = AircraftFrameViz()

	#node.publishWaypointMarkers()
	while rclpy.ok():
		try:
			# node.cool_example()
			rclpy.spin_once(aircraft_vis_node, timeout_sec=0.1)
			# rclpy.spin_once(node, timeout_sec=0.1)

		except KeyboardInterrupt:
			#kill node 
			aircraft_vis_node.get_logger().info('Shutting down')
			aircraft_vis_node.destroy_node()
			rclpy.shutdown()

if __name__=="__main__":
	main()    