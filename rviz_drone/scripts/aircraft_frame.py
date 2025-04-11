#!/usr/bin/env python3

"""
Broadcast static effector reference frame wrt to base frame of UAV
"""

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from drone_interfaces.msg import Telem, CtlTraj
from rviz_drone.rotation_utils import get_quaternion_from_euler
from rviz_drone.config import SCALE_SIM

import mavros
import time
from mavros.base import SENSOR_QOS

class AircraftFrame(Node):
    
        def __init__(self, 
                     pub_freq:int=30,
                     sub_freq:int=30,
                     sub_to_mavros:bool=True):
            super().__init__('aircraft_frame')

            # This is typically private attributes 
            self.pub_freq = pub_freq
            self.sub_freq = sub_freq
            self.sub_to_mavros = sub_to_mavros
            
            self.br = TransformBroadcaster(self)    
            self.declare_parameter('x', 0.0)
            self.declare_parameter('y', 0.0)
            self.declare_parameter('z', 0.0)

            self.declare_parameter('roll', 0.0)
            self.declare_parameter('pitch', 0.0)
            self.declare_parameter('yaw', 0.0)
            self.declare_parameter('parent_frame', 'map')
            self.declare_parameter('child_frame', 'aircraft_frame')
            self.declare_parameter('rate', 30.0)

            self.x = self.get_parameter('x').value
            self.y = self.get_parameter('y').value
            self.z = self.get_parameter('z').value

            self.roll = self.get_parameter('roll').value
            self.pitch = self.get_parameter('pitch').value
            self.yaw = self.get_parameter('yaw').value

            self.quaternion = get_quaternion_from_euler(
                float(self.roll), float(self.pitch), float(self.yaw))

            self.parent_frame = self.get_parameter('parent_frame').value
            self.child_frame = self.get_parameter('child_frame').value

            self.rate = self.get_parameter('rate').value
            self.timer = self.create_timer(1/self.rate, self.broadcastTransform)


            if self.sub_to_mavros:
                self.state_sub = self.create_subscription(mavros.local_position.Odometry,
                                                        'mavros/local_position/odom', 
                                                        self.mavros_state_callback, 
                                                        qos_profile=SENSOR_QOS)
            # TODO: map this with ardupilot telem message
            else:
                print('Subscribing to telem')        
                # self.state_sub = self.create_subscription(Telem, 
                #     'telem',
                #     self.state_callback,
                #     self.sub_freq)
        
        # These are all your methods
        def mavros_state_callback(self, msg:mavros.local_position.Odometry)->None:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.z = msg.pose.pose.position.z

            self.quaternion = [msg.pose.pose.orientation.x,
                               msg.pose.pose.orientation.y,
                               msg.pose.pose.orientation.z,
                               msg.pose.pose.orientation.w]
            
            start_time = time.time()
            self.broadcastTransform()
            print("time to compute is", time.time()- start_time)
        
        def broadcastTransform(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.child_frame
            t.transform.translation.x = self.x/SCALE_SIM
            t.transform.translation.y = self.y/SCALE_SIM
            t.transform.translation.z = self.z/SCALE_SIM
            t.transform.rotation.x = self.quaternion[0]
            t.transform.rotation.y = self.quaternion[1]
            t.transform.rotation.z = self.quaternion[2]
            t.transform.rotation.w = self.quaternion[3]
            
            position = [t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z]
            # print(f'position: {position}')
    
            self.br.sendTransform(t)    

def main(args=None):
    rclpy.init(args=args)

    node = AircraftFrame()

    rclpy.spin(node)

    #rclpy.shutdown()

if __name__ == '__main__':
    main()

