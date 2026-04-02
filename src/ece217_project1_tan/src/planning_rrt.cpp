#include <iostream>
#include <math.h>
#include "ece217_project1_tan/test_search_node.h"
#include <vector>
#include <deque>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unistd.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <chrono>

namespace ece217_project1_tan{

// funciton declaration
int twod_to_oned(int x, int y, int w);
double grid_to_meter(int a);
std::pair<int,int> oned_to_twod(int index, int w);
std::vector<int> c_space_expansion(std::vector<int>, int w, int h);
int meter_to_grid(double a);
std::shared_ptr<ece217_project1_tan::SearchNode_rrt> find_closest_node(std::vector<std::shared_ptr<ece217_project1_tan::SearchNode_rrt>>& closed_list , std::shared_ptr<ece217_project1_tan::SearchNode_rrt>& random_node);
std::shared_ptr<ece217_project1_tan::SearchNode_rrt> find_connected_node(std::shared_ptr<ece217_project1_tan::SearchNode_rrt>& closed_node, std::shared_ptr<ece217_project1_tan::SearchNode_rrt>& random_node, double set_d);
double random_double( double min, double max);


int process_map_rrt(const std::vector<int>& map_data, int width, int height) {

  // keep record of the start time of this script
  auto start = std::chrono::steady_clock::now();
  auto end = std::chrono::steady_clock::now();

  //set the distance the new node is to the closest explored node
  double set_d = 1;

  std::cout << std::endl << std::endl << "Start of RRT Search"<< std::endl;

  std::cout << std::endl;
  std::cout << "process_map_rrt receives width: " << width << std::endl;
  std::cout << "process_map_rrt receives height: " << height << std::endl;

  std::cout << "start of search_node_rrt"<< std::endl << std::endl;

  // c space expansion
  std::vector<int> expanded_map_data = c_space_expansion(map_data, width, height);

  // set up a publisher to publish the final_path
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("search_node_rrt_publisher");
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> path_publisher = node->create_publisher<nav_msgs::msg::Path>("plan_rrt",1);
  sleep(1);

  // create pointers for start node and goal node
  std::shared_ptr< ece217_project1_tan::SearchNode_rrt > start_node = std::make_shared< ece217_project1_tan::SearchNode_rrt >( -20.0, 0);
  std::shared_ptr< ece217_project1_tan::SearchNode_rrt > goal_node = std::make_shared< ece217_project1_tan::SearchNode_rrt >( 20.0, 0);

  // print start node and goal node
  std::cout << "start_node:" << *start_node << std::endl;
  std::cout << "goal_node:" << *goal_node << std::endl << std::endl;

  // create closed_list to store the nodes that have been explored.
  std::vector< std::shared_ptr< ece217_project1_tan::SearchNode_rrt > > closed_list;
  closed_list.push_back(start_node);

  // start creating random nodes and search
  std::shared_ptr<ece217_project1_tan::SearchNode_rrt> random_node = std::make_shared<ece217_project1_tan::SearchNode_rrt>();
  random_node->x = random_double(-25.6,25.6);
  random_node->y = random_double(-25.6,25.6);
  random_node->bp = nullptr;

  std::shared_ptr<ece217_project1_tan::SearchNode_rrt> closest_node = std::make_shared<ece217_project1_tan::SearchNode_rrt>();
  closest_node = find_closest_node(closed_list , random_node);

  std::shared_ptr<ece217_project1_tan::SearchNode_rrt> connected_node = std::make_shared<ece217_project1_tan::SearchNode_rrt>();
  connected_node = find_connected_node(closest_node, random_node, set_d);

  closed_list.push_back(connected_node);

  while(fabs((connected_node->x) - (goal_node->x))>1 || fabs((connected_node->y)-(goal_node->y))>1){
    //check the time cost: if it takes more than 10 seconds, print "didn't find goal" and exit the program
    end = std::chrono::steady_clock::now();
    if((end - start) > std::chrono::seconds(10) ){
      std::cout << "No goal is found within 10 seconds. Exit RRT search." << std::endl;

      return EXIT_FAILURE;
    }

    random_node->x = random_double(-25.6,25.6);
    random_node->y = random_double(-25.6,25.6);
    random_node->bp = nullptr;

    closest_node = find_closest_node(closed_list ,random_node);

    connected_node = find_connected_node(closest_node, random_node, set_d);

    int map_index = twod_to_oned(meter_to_grid(connected_node->x), meter_to_grid(connected_node->y), width);
    if(expanded_map_data[map_index] == -128){
      continue;
    }
    
    closed_list.push_back(connected_node);
  
  }

  // if goal is found, the while loop will end
  std::cout << "RRT Path is found!" << std::endl;  

  // create a vector to store all the memory of the nodes in the path
  std::deque< std::shared_ptr< ece217_project1_tan::SearchNode_rrt > > final_path;
  //create a new deque for posestamped messages
  std::deque<geometry_msgs::msg::PoseStamped> pose_path;

      
  //create a nav_msgs::msg::Path message
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = node->get_clock()->now();
  
  std::cout << "The path found by RRT" << std::endl;

  double rrt_cost=set_d;
  double delta_theta;

  while(connected_node->bp != nullptr){
    // print out the final path in the terminal
    final_path.push_front(connected_node);
          
    //std::cout << *connected_node << std::endl;
   
    //publish the path
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = connected_node->x;
    pose_stamped.pose.position.y = connected_node->y;
    pose_stamped.pose.position.z = 0.0;
    
    if(connected_node->bp->bp != nullptr){
      delta_theta = std::atan((connected_node->x - connected_node->bp->x)/(connected_node->y - connected_node->bp->y)) - std::atan((connected_node->bp->x - connected_node->bp->bp->x)/(connected_node->bp->y - connected_node->bp->bp->y));
      rrt_cost += sqrt(std::pow(connected_node->x - connected_node->bp->x, 2.0) + std::pow(connected_node->y - connected_node->bp->y, 2.0)) + delta_theta;

    }else if(connected_node->bp == nullptr){} 
        


    //rrt_cost += sqrt(std::pow(connected_node->x - connected_node->bp->x, 2.0) + std::pow(connected_node->y - connected_node->bp->y, 2.0)) + delta_theta;

    pose_path.push_front(pose_stamped);
    connected_node = connected_node->bp;
  }

  
  end = std::chrono::steady_clock::now();
  std::cout << "Time used for RRT search is " << std::chrono::duration<double>(end - start).count() << "s" << std::endl;
  std::cout << "Cost for RRT search is " << rrt_cost << std::endl;
 

  path_msg.poses = std::vector<geometry_msgs::msg::PoseStamped>(pose_path.begin(), pose_path.end());
  //publish the path
  path_publisher->publish(path_msg);
  
  rclcpp::spin_some(node);  
  sleep(2);
  rclcpp::spin_some(node);

  return EXIT_SUCCESS;
}


std::shared_ptr<ece217_project1_tan::SearchNode_rrt> find_closest_node(std::vector<std::shared_ptr<ece217_project1_tan::SearchNode_rrt>>& closed_list , std::shared_ptr<ece217_project1_tan::SearchNode_rrt>& random_node){

  double shortest_distance = 51.2;
  std::shared_ptr<ece217_project1_tan::SearchNode_rrt> closest_node;
  double distance;

  for(auto closed_node : closed_list){
    distance = sqrt(std::pow(random_node->x - closed_node->x,2.0) + std::pow(random_node->y - closed_node->y, 2.0));
    if(distance < shortest_distance){
      shortest_distance = distance;
      closest_node = closed_node;
    }	    

  }  

  return closest_node;
}	


std::shared_ptr<ece217_project1_tan::SearchNode_rrt> find_connected_node(std::shared_ptr<ece217_project1_tan::SearchNode_rrt>& closest_node, std::shared_ptr<ece217_project1_tan::SearchNode_rrt>& random_node, double set_d){
 
  std::shared_ptr<ece217_project1_tan::SearchNode_rrt> connected_node = std::make_shared<ece217_project1_tan::SearchNode_rrt>();
  
  double big_diagonal = sqrt(std::pow(random_node->x - closest_node->x,2.0) + std::pow(random_node->y - closest_node->y, 2.0));
  double big_x = random_node->x - closest_node->x;
  double big_y = random_node->y - closest_node->y;

  connected_node->x = closest_node->x + set_d * big_x / big_diagonal ; 
  connected_node->y = closest_node->y + set_d * big_y / big_diagonal ;
  connected_node->bp = closest_node ;

  return connected_node;
}

  double random_double( double min, double max){
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dist(min,max);

    return dist(gen);
  }
}
