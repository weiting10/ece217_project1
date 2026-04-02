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

namespace ece217_project1_tan{

// funciton declaration
int twod_to_oned(int x, int y, int w);
double grid_to_meter(int a);
std::pair<int,int> oned_to_twod(int index, int w);
std::vector<int> c_space_expansion(std::vector<int>, int w, int h);
int meter_to_grid(double a);

int process_map(const std::vector<int>& map_data, int width, int height) {
  // keep record of the start time of this script
  auto start = std::chrono::steady_clock::now();
  auto end = std::chrono::steady_clock::now();

  std::cout << "Start processing map"<< std::endl;

  std::cout << std::endl;
  std::cout << "process_map receives width: " << width << std::endl;
  std::cout << "process_map receives height: " << height << std::endl << std::endl;

  std::cout << "Start of A Star Search"<< std::endl << std::endl;

  // c space expansion
  std::vector<int> expanded_map_data = c_space_expansion(map_data, width, height);

  // set up a publisher to publish the final_path
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_search_node_publisher");
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> path_publisher = node->create_publisher<nav_msgs::msg::Path>("plan",1);
  sleep(1);

  // create pointers for start node and goal node
  std::shared_ptr< ece217_project1_tan::SearchNode > start_node = std::make_shared< ece217_project1_tan::SearchNode >( meter_to_grid(-20.0), meter_to_grid(0));
  std::shared_ptr< ece217_project1_tan::SearchNode > goal_node = std::make_shared< ece217_project1_tan::SearchNode >( meter_to_grid(20), meter_to_grid(0));

  // compute the h and f for start node
  start_node->h = 0.2* (sqrt( std::pow( goal_node->x - start_node->x, 2.0 ) + std::pow( goal_node->y - start_node->y, 2.0 ) ));
  start_node->f = start_node->g + start_node->h;

  // print start node and goal node
  std::cout << "start_node:" << *start_node << std::endl;
  std::cout << "goal_node:" << *goal_node << std::endl << std::endl;

  // create open_list and closed_list and put start node into open_list
  std::deque< std::shared_ptr< ece217_project1_tan::SearchNode > > open_list;
  open_list.push_back( start_node );
  std::vector< std::shared_ptr< ece217_project1_tan::SearchNode > > closed_list;

  // creat shared pointer top
  std::shared_ptr< ece217_project1_tan::SearchNode > top = nullptr;

  // while the open_list isn't empty
  while( open_list.empty() == false ){
   	 
    //check the time cost: if it takes more than 10 seconds, print "didn't find goal" and exit the program
    end = std::chrono::steady_clock::now();
    if((end - start) > std::chrono::seconds(10) ){
      std::cout << "No goal is found within 10 seconds. Exit RRT search." << std::endl;

      return EXIT_FAILURE;
    }

    // get the node with smallest f 
    top = open_list.front();

    // check if the top node is the goal by checking if h==zero
    if(top->h <= 0.2){
      std::cout << "Goal is found!" << std::endl;
      
      // create a vector to store all the memory of the nodes in the path
      std::deque< std::shared_ptr< ece217_project1_tan::SearchNode > > final_path;

      //create a nav_msgs::msg::Path message
      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = "map";

      //create a new deque for posestamped messages
      std::deque<geometry_msgs::msg::PoseStamped> pose_path;

      end = std::chrono::steady_clock::now();
      std::cout << "Time used for A Star Search is " << std::chrono::duration<double>(end - start).count() << "s" << std::endl;
      std::cout << "Cost for A Star Search is " << top->g << std::endl;

      while(top != start_node->bp){
	// print out the final path in the terminal
	final_path.push_front(top);   
	
	//publish the path
	geometry_msgs::msg::PoseStamped pose_stamped;
	pose_stamped.header.frame_id = "map";
	pose_stamped.pose.position.x = grid_to_meter(top->x);
	pose_stamped.pose.position.y = grid_to_meter(top->y);
	pose_stamped.pose.position.z = 0.0;

	pose_path.push_front(pose_stamped);
	top = top->bp;
      }

      path_msg.poses = std::vector<geometry_msgs::msg::PoseStamped>(pose_path.begin(), pose_path.end());
      //publish the path
      path_publisher->publish(path_msg);

      sleep(2);
	  
      return 0;
      
    }
    
    // pop the first node in the open_list and save top to the closed_list
    open_list.pop_front();
    closed_list.push_back(top);

    // create descendents
    std::shared_ptr< ece217_project1_tan::SearchNode > a = std::make_shared< ece217_project1_tan::SearchNode>( top->x , top->y+1, M_PI/2 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< ece217_project1_tan::SearchNode > b = std::make_shared< ece217_project1_tan::SearchNode>( top->x , top->y-1, 3*M_PI/2 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< ece217_project1_tan::SearchNode > c = std::make_shared< ece217_project1_tan::SearchNode>( top->x+1, top->y, 0.0 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< ece217_project1_tan::SearchNode > d = std::make_shared< ece217_project1_tan::SearchNode>( top->x-1, top->y, M_PI , 0.0, 0.0, 0.0, top);
    std::shared_ptr< ece217_project1_tan::SearchNode > m = std::make_shared< ece217_project1_tan::SearchNode>( top->x+1, top->y+1, M_PI/4 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< ece217_project1_tan::SearchNode > n = std::make_shared< ece217_project1_tan::SearchNode>( top->x+1, top->y-1, -M_PI/4 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< ece217_project1_tan::SearchNode > p = std::make_shared< ece217_project1_tan::SearchNode>( top->x-1, top->y+1, 3*M_PI/4 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< ece217_project1_tan::SearchNode > q = std::make_shared< ece217_project1_tan::SearchNode>( top->x-1, top->y-1, 5*M_PI/4 , 0.0, 0.0, 0.0, top);

    // compute the distance from descendents to the goal (h)
    a->h = 0.2*(sqrt( std::pow( goal_node->x - a->x, 2.0 ) + std::pow( goal_node->y - a->y, 2.0 )) ) + fabs(goal_node->theta - a->theta);
    b->h = 0.2*(sqrt( std::pow( goal_node->x - b->x, 2.0 ) + std::pow( goal_node->y - b->y, 2.0 )) ) + fabs(goal_node->theta - b->theta);
    c->h = 0.2*(sqrt( std::pow( goal_node->x - c->x, 2.0 ) + std::pow( goal_node->y - c->y, 2.0 )) ) + fabs(goal_node->theta - c->theta);
    d->h = 0.2*(sqrt( std::pow( goal_node->x - d->x, 2.0 ) + std::pow( goal_node->y - d->y, 2.0 )) ) + fabs(goal_node->theta - d->theta);
    m->h = 0.2*(sqrt( std::pow( goal_node->x - m->x, 2.0 ) + std::pow( goal_node->y - m->y, 2.0 )) ) + fabs(goal_node->theta - m->theta);
    n->h = 0.2*(sqrt( std::pow( goal_node->x - n->x, 2.0 ) + std::pow( goal_node->y - n->y, 2.0 )) ) + fabs(goal_node->theta - n->theta);
    p->h = 0.2*(sqrt( std::pow( goal_node->x - p->x, 2.0 ) + std::pow( goal_node->y - p->y, 2.0 )) ) + fabs(goal_node->theta - p->theta);
    q->h = 0.2*(sqrt( std::pow( goal_node->x - q->x, 2.0 ) + std::pow( goal_node->y - q->y, 2.0 )) ) + fabs(goal_node->theta - q->theta);
   
    // compute the distance from descendents to the start (g)
    a->g = 0.2 + abs(a->theta - a->bp->theta) + a->bp->g;
    b->g = 0.2 + abs(b->theta - b->bp->theta) + b->bp->g;
    c->g = 0.2 + abs(c->theta - c->bp->theta) + c->bp->g;
    d->g = 0.2 + abs(d->theta - d->bp->theta) + d->bp->g;
    m->g = 0.2828 + abs(m->theta - m->bp->theta) + m->bp->g;
    n->g = 0.2828 + abs(n->theta - n->bp->theta) + n->bp->g;
    p->g = 0.2828 + abs(p->theta - p->bp->theta) + p->bp->g;
    q->g = 0.2828 + abs(q->theta - q->bp->theta) + q->bp->g;
  
    // compute f by adding h and g for each descendent
    a->f = a->g + 1.4* a->h;
    b->f = b->g + 1.4* b->h;
    c->f = c->g + 1.4* c->h;
    d->f = d->g + 1.4* d->h;
    m->f = m->g + 1.4* m->h;
    n->f = n->g + 1.4* n->h;
    p->f = p->g + 1.4* p->h;
    q->f = q->g + 1.4* q->h;
    
    // create a vector to contain a,b,c,d
    std::vector<std::shared_ptr<ece217_project1_tan::SearchNode>> children = {a,b,c,d,m,n,p,q};

    // check the four descendents
    for(auto& child:children){
      // check if each descendent is out of the boundary
      if((child->x <0)||(child->x >= width)||(child->y <0)||(child->y >= height)){
        //closed_list.push_back(child);
        continue;
      }

      // check if each descendent is obstacle
      int map_index = twod_to_oned(child->x, child->y, width);
      if(expanded_map_data[map_index] == -128){
        //closed_list.push_back(child);
	continue;
      }

      // check if each descendent is in the closed_list	    
      bool in_closed_list=0;
      for(auto& closed_node:closed_list){
        if((child->x == closed_node->x) && (child->y == closed_node->y) && (fabs(child->theta - closed_node->theta)<0.2)){
	  in_closed_list=1;
	  break;
        }
      }

      // check if each descendent is in the open_list
      bool in_open_list=0;
      for(auto& open_node:open_list){
        if((child->x == open_node->x)  && (child->y == open_node->y)  && (fabs(child->theta - open_node->theta)<0.2)){
          in_open_list=1;
          break;
        }    

      }

      if(in_closed_list == 0){
        if(in_open_list == 0){
          open_list.push_back(child);
        }
      }
    }

  // sort the open list
    std::sort( open_list.begin(), open_list.end(), []( const std::shared_ptr< ece217_project1_tan::SearchNode >& firstArg, const std::shared_ptr< ece217_project1_tan::SearchNode >& secondArg ) { return ( firstArg->f < secondArg->f ); } );
    }
    
    std::cout << "No path is found" << std::endl;
    return 0;
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


}

