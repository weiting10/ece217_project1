#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

void subscriber_callback(geometry_msgs::msg::Pose2D::UniquePtr msg){

  std::cout << "in subscriber_callback" << std::endl;
  std::cout << "msg->x:" << msg->x << " msg->y:" << msg->y << " msg->theta:" << msg->theta << std::endl;

}
  

int main( int argc, char* argv[] ){
  
  std::cout<< "start of ece217_project1 test_subscriber" << std::endl;

  rclcpp::init( argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared( "test_subscriber");

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose2D>> pose2d_subscriber = node->create_subscription<geometry_msgs::msg::Pose2D>("pose2d",1,subscriber_callback);

  rclcpp::spin(node);
  rclcpp::shutdown();

  std::cout<< "end of ece217_project1 test_subscriber" << std::endl;

  return 0;
}
