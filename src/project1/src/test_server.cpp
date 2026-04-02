#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "project1/srv/planning_query.hpp"
#include "project1/test_search_node.h"

void service_callback(const std::shared_ptr< project1::srv::PlanningQuery::Request> request,
		const std::shared_ptr< project1::srv::PlanningQuery::Response> response){
  RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Incoming PlanningQuery::Request");
  
  //printing the map data out
  std::cout <<"request start x:"<<request->start.x <<" y:" << request->start.y << " theta:" << request->start.theta << std::endl;
  std::cout <<"request goal x:"<<request->goal.x <<" y:" << request->goal.y << " theta:" << request->goal.theta << std::endl;
  std::cout <<"request map frame id:" << request->map.header.frame_id << std::endl;
  std::cout << "request map width: " << request->map.info.width <<std::endl;
  std::cout << "request map height: " << request->map.info.height << std::endl;
  std::cout << "request map data:" <<  std::endl;

  // store the data of the map to map_data
  std::vector<int> map_data;
  for(auto val : request->map.data){
    map_data.push_back(static_cast<int>(val));
  }

  int width = request->map.info.width;
  int height = request->map.info.height;
  
  //pass the map data, width , and height to the process_map function, and generate a path
  project1::process_map(map_data, width, height);
  std::cout << std::endl;

  project1::process_map_rrt(map_data, width, height);
  std::cout << std::endl;

  RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Outgoing PlanningQuery::Response:");


  return;
}

int main( int argc, char* argv[]){

  rclcpp::init(argc,argv);


  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("project1_server");

  rclcpp::Service<project1::srv::PlanningQuery>::SharedPtr service = node->create_service<project1::srv::PlanningQuery>( "planning_query", &service_callback);

  RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Ready to generate plans.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return EXIT_SUCCESS; 
}
