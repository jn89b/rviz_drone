#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from drone_interfaces.msg import CtlTraj
from rviz_drone.config import SCALE_SIM
from ros_mpc.aircraft_config import obs_avoid_params

"""
#TODO:
Not a good idea to put it in this package need to put it in the rviz_drone package
but have to grab the random seeded obstacles from the ros_mpc package, future to do
"""
print("scale sim: ", SCALE_SIM)


class ObstacleViz(Node):
    def __init__(self):
        super().__init__("obstacle_visualizer")
        
        self.marker_pub = self.create_publisher(MarkerArray, 
                                                "obstacle_marker", 
                                                10)
        
        self.obs_avoid_params = obs_avoid_params
        self.id_counter = 0
        
    def show_obstacles(self) -> None:
        x = self.obs_avoid_params['x']
        y = self.obs_avoid_params['y']
        radii = self.obs_avoid_params['radii']
        # if len(x) == 0:
        #     print("No obstacles to visualize")
        
        self.marker_array = MarkerArray()
        
        for i, (x_, y_, r) in enumerate(zip(x, y, radii)):
            marker = Marker()
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.color.r = 0.58
            marker.color.g = 0.29
            marker.color.b = 0.0
            marker.color.a = 1.0    
            marker.pose.position.x = x_/SCALE_SIM
            marker.pose.position.y = y_/SCALE_SIM
            marker.pose.position.z = 0.0
            
            marker.id = self.id_counter
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "/map"
            marker.ns = "obstacles" + str(self.id_counter)
            marker.scale.x = 2.0*r/SCALE_SIM
            marker.scale.y = 2.0*r/SCALE_SIM
            marker.scale.z = 5.0

            self.marker_array.markers.append(marker)
            self.id_counter += 1
                    
        self.marker_pub.publish(self.marker_array)
        
def main()->None:
    rclpy.init()
    obs_viz = ObstacleViz()

    #set the rate to be every 1000 seconds
    obs_viz.show_obstacles()
    time_interval = 10000
    while rclpy.ok():
        pass
        # obs_viz.show_obstacles()
        

    rclpy.shutdown()
    
if __name__=="__main__":
    main()    
            
        