#ifndef CPP_AIRCRAFT_FRAME
#define CPP_AIRCRAFT_FRAME

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "drone_interfaces/msg/telem.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

// #include <Eigen/Dense>

class AircraftFrame : public rclcpp::Node
{
public:
  AircraftFrame(); // Constructor
  ~AircraftFrame(); // Destructor
  rclcpp::Subscription<drone_interfaces::msg::Telem>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Methods
  void timer_callback();
  // void state_callback();
  void state_callback(const drone_interfaces::msg::Telem::SharedPtr msg);
  void broadcast_transform();


 private:

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  const float SCALE_SIM = 50;

  // size_t count_;
  // Member variables for parameters
  double x_, y_, z_;
  double roll_, pitch_, yaw_;
  std::string parent_frame_;
  std::string child_frame_;
  double rate_;
  double qx_, qy_, qz_, qw_;
  geometry_msgs::msg::TransformStamped t;

};

#endif // CPP_MINIMAL_PUBLISHER_H