#include "aircraft_frame_c.hpp"
// float add(x,y) 
// {

// }


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AircraftFrame>());
  rclcpp::shutdown();
  return 0;
}