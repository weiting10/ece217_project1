#include <iostream>
#include <math.h>
#include "project1/test_search_node.h"
#include <vector>
#include <deque>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unistd.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace project1{

// funciton declaration
int twod_to_oned(int x, int y, int w);
double grid_to_meter(int a);
std::pair<int,int> oned_to_twod(int index, int w);
std::vector<int> c_space_expansion(std::vector<int>, int w, int h);
int meter_to_grid(double a);
std::shared_ptr<project1::SearchNode_rrt> find_closest_node(std::vector<project1::SearchNode> closed_list , std::shared_ptr<project1::SearchNode_rrt> random_node);
std::shared_ptr<project1::SearchNode_rrt> find_connected_node(project1::SearchNode_rrt closed_node, project1::SearchNode_rrt random_node);


int process_map_rrt(const std::vector<int>& map_data, int width, int height) {

  //set the distance the new node is to the closest explored node
  double set_d = 1;

  std::cout << "Start processing map"<< std::endl;

  /*
  for(int i = 0; i< map_data.size(); i++){
    std::cout << map_data[i] << " ";
  } */

  std::cout << std::endl;
  std::cout << "process_map receives width: " << width << std::endl;
  std::cout << "process_map receives height: " << height << std::endl;

  std::cout << "start of test_search_node"<< std::endl << std::endl;

  // c space expansion
  std::vector<int> expanded_map_data = c_space_expansion(map_data, width, height);

  // set up a publisher to publish the final_path
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_search_node_publisher");
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> path_publisher = node->create_publisher<nav_msgs::msg::Path>("plan",1);
  sleep(1);

  // create pointers for start node and goal node
  std::shared_ptr< project1::SearchNode_rrt > start_node = std::make_shared< project1::SearchNode_rrt >( -20.0, 0);
  std::shared_ptr< project1::SearchNode > goal_node = std::make_shared< project1::SearchNode >( 20.0, 0);

  // print start node and goal node
  std::cout << "start_node:" << *start_node << std::endl;
  std::cout << "goal_node:" << *goal_node << std::endl << std::endl;

  // create closed_list to store the nodes that have been explored.
  std::vector< std::shared_ptr< project1::SearchNode_rrt > > closed_list;
  closed_list.push_back(start_node);

  std::shared_ptr<project1::SearchNode_rrt> closest_node = find_closest_node(std::vector<project1::SearchNode> closed_list , std::shared_ptr<project1::SearchNode_rrt> random_node);

  std::shared_ptr<project1::SearchNode_rrt> connected_node = find_connected_node(std::shared_ptr<project1::SearchNode_rrt> closest_node, std::shared_ptr<project1::SearchNode_rrt> random_node);

  closed_list.push_back(connected_node);
	


}


std::shared_ptr<project1::SearchNode_rrt> find_closest_node(std::vector<project1::SearchNode> closed_list , std::shared_ptr<project1::SearchNode_rrt> random_node){

  double shortest_distance = 51.2;
  std::shared_ptr<project1::SearchNode_rrt> closest_node;
  double distance;

  for(auto closed_node : closed_list){
    distance = sqrt(std::pow(random_node->x - closed_node->x,2.0) + std::pow(random_node->y - closed_node->y), 2.0);
    if(distance < shortest_distance){
      shortest_distance = distance;
      closest_node = closed_node;
    }	    

  }  

  return closest_node;
}	


std::shared_ptr<project1::SearchNode_rrt> find_connected_node(std::shared_ptr<project1::SearchNode_rrt> closest_node, std::shared_ptr<project1::SearchNode_rrt> random_node){
 
  std::shared_ptr<project1::SearchNode_rrt> connected_node;
  
  double big_diagonal = sqrt(std::pow(random_node->x - closest_node->x,2.0) + std::pow(random_node->y - closest_node->y), 2.0);
  double big_x = random_node->x - closest_node->x;
  double big_y = random_node->y - closest_node->y;

  connected_node->x = closest_node->x + set_d * big_x / big_diagonal ; 
  connected_node->y = closest_node->y + set_d * big_y / big_diagonal ;
  connected_node->bp = closest_node ;

  return connected_node;
}







  // c space expansion: assuming 0 is the obstacles

  std::vector<int> c_space_expansion(std::vector<int> map_data, int w, int h){
    std::vector<int> expanded_obstacle(w*h, 127);

    int x;
    int y;
    int index;

    for(int p=0; p < w*h ; p++){
      int node = map_data[p];
      if(node == -128){
        auto [x,y] = oned_to_twod(p,w);
        for(int i=0; i<3 ;i++){
          if(((x-1+i)<0)||((x-1+i)>(w-1))){
            continue;
          }

          for(int j=0; j<3 ;j++){
            if(((y-1+j)<0)||((y-1+j)>(h-1))){
              continue;
            }

            index = twod_to_oned(x-1+i,y-1+j,w);
            expanded_obstacle[index]=-128;

          }
        }
      }
    }
    return expanded_obstacle;
  }


  // map (x,y) to 1D array index
  int twod_to_oned(int x, int y, int w){
    int index = y*w + x ;
    return index;
  }

  // map 1D array index to (x,y)
  std::pair<int,int> oned_to_twod(int index, int w){
    int x;
    int y;

    y = index/w;
    x = index%w;

    return {x,y};
  }

  // convert from grid index to distance in meter
  double grid_to_meter(int a){

    double distance;
    distance = static_cast<double>(a) * 0.2 - 25.6;

    return distance;
  }

  int meter_to_grid(double a){
    int grid;
    grid = (a + 25.6)/0.2;
    return grid;
  }


