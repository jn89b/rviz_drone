#include "aircraft_frame_c.hpp"

AircraftFrame::AircraftFrame(): Node("aircraft_frame_cpp")
{
    // Declare parameters with default values similar to the Python version:
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("z", 0.0);

    this->declare_parameter<double>("roll", 0.0);
    this->declare_parameter<double>("pitch", 0.0);
    this->declare_parameter<double>("yaw", 0.0);

    this->declare_parameter<std::string>("parent_frame", "map");
    this->declare_parameter<std::string>("child_frame", "aircraft_frame");

    this->declare_parameter<double>("rate", 30.0);

    // Retrieve the parameters, assigning them to class member variables
    x_ = this->get_parameter("x").as_double();
    y_ = this->get_parameter("y").as_double();
    z_ = this->get_parameter("z").as_double();

    roll_ = this->get_parameter("roll").as_double();
    pitch_ = this->get_parameter("pitch").as_double();
    yaw_ = this->get_parameter("yaw").as_double();

    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();

    rate_ = this->get_parameter("rate").as_double();

    // Optionally, if you wish to broadcast transforms as in your Python node:
    // br_ = tf2_ros::TransformBroadcaster(this);

    // Initialize publisher and timer as in your example
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),   // Adjust the timer duration as needed
        std::bind(&AircraftFrame::timer_callback, this)
    );

    subscription_ = this->create_subscription<drone_interfaces::msg::Telem>(
        "telem", 10, std::bind(&AircraftFrame::state_callback, this, std::placeholders::_1));

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

}


void AircraftFrame::timer_callback()
{
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, world! " + std::to_string(count_++);
//   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   publisher_->publish(message);

  // If you were using the TransformBroadcaster, here you could publish the transform
}

void AircraftFrame::state_callback(const drone_interfaces::msg::Telem::SharedPtr msg)
{
    x_ = msg->x;
    y_ = msg->y;
    z_ = msg->z; 
    qx_ = msg->qx;
    qy_ = msg->qy;
    qz_ = msg->qz;
    qw_ = msg->qw;
    // std::cout << "x: " << x_ << std::endl;
    // std::cout << "y: " << y_ << std::endl;
    // std::cout << "z: " << z_ << std::endl;
    // std::cout << "qx: " << qx_ << std::endl;
    // std::cout << "qy: " << qy_ << std::endl;
    // std::cout << "qz: " << qz_ << std::endl;
    // std::cout << "qw: " << qw_ << std::endl;
    // Capture the start time before sending the transform.
    auto start_time = std::chrono::steady_clock::now();
    broadcast_transform(); 
    // Capture the end time after sending the transform.
    auto end_time = std::chrono::steady_clock::now();
    // Calculate the elapsed time in seconds as a floating-point value
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "time: " << elapsed.count() << std::endl;
}

void AircraftFrame::broadcast_transform()
{
    // geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent_frame_;
    t.child_frame_id = child_frame_;

    t.transform.translation.x = x_/SCALE_SIM;
    t.transform.translation.y = y_/SCALE_SIM;
    t.transform.translation.z = z_/SCALE_SIM;

    t.transform.rotation.x = qx_;    
    t.transform.rotation.y = qy_;
    t.transform.rotation.z = qz_;
    t.transform.rotation.w = qw_;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // std::cout << "scaled x " << t.transform.translation.x << std::endl;
    // std::cout << "scaled y " << t.transform.translation.y << std::endl;
    // std::cout << "scaled z " << t.transform.translation.z << std::endl;

}

AircraftFrame::~AircraftFrame()
{
  // Destructor - cleanup if needed
}
