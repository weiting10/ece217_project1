#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <boost/program_options.hpp>
#include "ece217_project1_tan/srv/planning_query.hpp"
#include <fstream>
#include <chrono>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "ece217_project1_tan/srv/planning_query.hpp"

void request_to_visualization_msgs_marker_array( const std::shared_ptr<rclcpp::Node>& node, const std::shared_ptr< ece217_project1_tan::srv::PlanningQuery::Request >& request, visualization_msgs::msg::MarkerArray& markerArray ){

  // start_marker
  visualization_msgs::msg::Marker start_marker;
  start_marker.header.frame_id = "map";
  start_marker.header.stamp = node->get_clock()->now();
  start_marker.id = 0;
  start_marker.ns = "request";
  start_marker.type = visualization_msgs::msg::Marker::SPHERE;
  start_marker.action = visualization_msgs::msg::Marker::ADD;
  start_marker.pose.position.x = request->start.x;
  start_marker.pose.position.y = request->start.y;
  start_marker.pose.position.z = 0.0;
  start_marker.pose.orientation.x = 0.0;
  start_marker.pose.orientation.y = 0.0;
  start_marker.pose.orientation.z = 0.0;
  start_marker.pose.orientation.w = 1.0;
  start_marker.scale.x = 1.0;
  start_marker.scale.y = 1.0;
  start_marker.scale.z = 1.0;
  start_marker.color.r = 1.0;
  start_marker.color.g = 0.0;
  start_marker.color.b = 0.0;
  start_marker.color.a = 1.0;
  start_marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
  markerArray.markers.push_back( start_marker );

  // goal_marker
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = node->get_clock()->now();
  goal_marker.id = 1;
  goal_marker.ns = "request";
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position.x = request->goal.x;
  goal_marker.pose.position.y = request->goal.y;
  goal_marker.pose.position.z = 0.0;
  goal_marker.pose.orientation.x = 0.0;
  goal_marker.pose.orientation.y = 0.0;
  goal_marker.pose.orientation.z = 0.0;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.scale.x = 1.0;
  goal_marker.scale.y = 1.0;
  goal_marker.scale.z = 1.0;
  goal_marker.color.r = 0.0;
  goal_marker.color.g = 1.0;
  goal_marker.color.b = 0.0;
  goal_marker.color.a = 1.0;
  goal_marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
  markerArray.markers.push_back( goal_marker );

  return;
} 

void nav_msgs_occupancy_grid_from_file( const std::string& filename, nav_msgs::msg::OccupancyGrid& msg ){

  std::ifstream infile;
  infile.open( filename );

  std::string tmp;
  std::getline( infile, tmp ); // identifier
  std::getline( infile, tmp ); // comment
  int width = 0;
  int height = 0;
  int max_value = 0;
  infile >> width;
  infile >> height;
  infile >> max_value;
  msg.info.resolution = 0.2;
  msg.info.width = width;
  msg.info.height = height;
  msg.info.origin.position.x = -128*0.2;
  msg.info.origin.position.y = -128*0.2;
  msg.info.origin.position.z = 0.0;;
  msg.info.origin.orientation.x = 0.0;;
  msg.info.origin.orientation.y = 0.0;;
  msg.info.origin.orientation.z = 0.0;;
  msg.info.origin.orientation.w = 1.0;;
  msg.data.resize( msg.info.width * msg.info.height );

  std::vector< int8_t > tmp_data;
  for( int row = 0; row < msg.info.height; row++ ){
    for( int col = 0; col < msg.info.width; col++ ){
      int index = msg.info.width * ( msg.info.height - row - 1 ) + ( col );
      int value;
      infile >> value;
      msg.data[index] = ( value - 128 );
    }
  }
  return;

} 


int main( int argc, char* argv[]){

  boost::program_options::options_description desc("Options");
  desc.add_options()("help","Help Screen")
	        ("map", boost::program_options::value<std::string>(), "map file")
	  	("start-x", boost::program_options::value<double>()->default_value(-20.0),"start x value")
	  	("start-y", boost::program_options::value<double>()->default_value(0.0),"start y value")
	  	("start-theta", boost::program_options::value<double>()->default_value(0.0),"start theta value")
	  	("goal-x", boost::program_options::value<double>()->default_value(20.0),"goal x value")
	  	("goal-y", boost::program_options::value<double>()->default_value(0.0),"goal y value")
	  	("goal-theta", boost::program_options::value<double>()->default_value(0.0),"goal theta value")
    		("output", boost::program_options::value<std::string>(), "output file");

  boost::program_options::variables_map vm;
  boost::program_options::store(
  boost::program_options::parse_command_line( argc, argv, desc ), vm );
  boost::program_options::notify(vm);

  if(vm.count("help")){
    std::cout<< desc << std::endl;
    return EXIT_SUCCESS;
  }

  rclcpp::init( argc, argv);

  std::shared_ptr< rclcpp::Node > node = rclcpp::Node::make_shared("ece217_project1_tan_client");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto occupancy_grid_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", qos);
	
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray_<std::allocator<void> >, std::allocator<void> > > marker_array_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 1 );

  rclcpp::Client<ece217_project1_tan::srv::PlanningQuery>::SharedPtr client = node->create_client<ece217_project1_tan::srv::PlanningQuery>("planning_query");

  std::shared_ptr< ece217_project1_tan::srv::PlanningQuery::Request > request = std::make_shared< ece217_project1_tan::srv::PlanningQuery::Request>();

  request->start.x = vm["start-x"].as<double>();
  request->start.y = vm["start-y"].as<double>();
  request->start.theta = vm["start-theta"].as<double>();

  request->goal.x = vm["goal-x"].as<double>();
  request->goal.y = vm["goal-y"].as<double>();
  request->goal.theta = vm["goal-theta"].as<double>();
  
  request->map.header.frame_id = "map";
  request->map.header.stamp = node->get_clock()->now();

  if( vm.count("map") ){
    RCLCPP_INFO_STREAM( rclcpp::get_logger("rclcpp"), "READING MAP FROM " << vm["map"].as<std::string>() );
    nav_msgs_occupancy_grid_from_file( vm["map"].as<std::string>(), request->map );
  } else {
    RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "No map provided, exiting" );
    return EXIT_FAILURE; 
  }

  while( !client->wait_for_service( std::chrono_literals::operator""s(1))){
    if( !rclcpp::ok()) {
      RCLCPP_ERROR( rclcpp::get_logger( "rclcpp" ), "Interrupted while waiting for service, exiting");
      return EXIT_FAILURE;
    }
    RCLCPP_INFO( rclcpp::get_logger( "rclcpp" ), "service not available, waiting...");
  }

  visualization_msgs::msg::MarkerArray marker_array;
  request_to_visualization_msgs_marker_array( node, request, marker_array );

  rclcpp::Client<ece217_project1_tan::srv::PlanningQuery>::FutureAndRequestId result = client->async_send_request( request );
  if( rclcpp::spin_until_future_complete( node, result ) == rclcpp::FutureReturnCode::SUCCESS ){
    RCLCPP_INFO( rclcpp::get_logger( "rclcpp" ), "Success in calling service" );
    occupancy_grid_publisher->publish( request->map );
    marker_array_publisher->publish( marker_array );
  } else{
    RCLCPP_INFO( rclcpp::get_logger( "rclcpp" ), "Failure in calling service" );
  }

  RCLCPP_INFO( rclcpp::get_logger( "rclcpp" ), "Client program complete");

  rclcpp::shutdown();
  
  return EXIT_SUCCESS;
}

