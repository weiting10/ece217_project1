#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <unistd.h>
#include <boost/program_options.hpp>

int main(int argc, char* argv[])
{
  std::cout << "start of project1 test_publisher" << std::endl;
  boost::program_options::options_description desc{"Option"};
  desc.add_options()("help","Help screen")(
    "x", boost::program_options::value<double>()->default_value(1.0), "x value (m)")(
    "y", boost::program_options::value<double>()->default_value(2.0), "y value (m)")(
    "theta", boost::program_options::value<double>()->default_value(M_PI), "theta value (rad)");

  boost::program_options::variables_map vm;
  boost::program_options::store( boost::program_options::parse_command_line( argc, argv, desc), vm);
  boost::program_options::notify(vm);
  if (vm.count("help")){
    std::cout << desc <<std::endl;
    return 0;
  }

  rclcpp::init(argc,argv) ;
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_publisher");

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Pose2D>> pose2d_publisher = node->create_publisher<geometry_msgs::msg::Pose2D>("pose2d",1);

  sleep(1);
  geometry_msgs::msg::Pose2D pose2d_msg;
  pose2d_msg.x = vm["x"].as<double>();
  pose2d_msg.y = vm["y"].as<double>();
  pose2d_msg.theta= vm["theta"].as<double>();

  std::cout << "x:" << pose2d_msg.x<< " y:" << pose2d_msg.y << " theta:"<< pose2d_msg.theta<< std::endl;

  pose2d_publisher->publish(pose2d_msg);


  std::cout << "end of project1 test_publisher" << std::endl;
  
  return 0;
}
