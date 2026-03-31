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

int twod_to_oned(double x, double y, int w);
std::pair<int,int> oned_to_twod(int index, int w);
std::vector<int> c_space_expansion(std::vector<int>, int w, int h);

int process_map(const std::vector<int>& map_data, int width, int height) {
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
  std::shared_ptr< project1::SearchNode > start_node = std::make_shared< project1::SearchNode >( -20, 0);
  std::shared_ptr< project1::SearchNode > goal_node = std::make_shared< project1::SearchNode >(20, 0);

  // compute the h and f for start node
  start_node->h = sqrt( std::pow( goal_node->x - start_node->x, 2.0 ) + std::pow( goal_node->y - start_node->y, 2.0 ) );
  start_node->f = start_node->g + start_node->h;

  // print start node and goal node
  std::cout << "start_node:" << *start_node << std::endl;
  std::cout << "goal_node:" << *goal_node << std::endl << std::endl;

  // create open_list and closed_list and put start node into open_list
  std::deque< std::shared_ptr< project1::SearchNode > > open_list;
  open_list.push_back( start_node );
  std::vector< std::shared_ptr< project1::SearchNode > > closed_list;

  //std::cout << "open_list" << open_list << std::endl;
  //std::cout << "closed_list" << closed_list << std::endl;

  // creat shared pointer top
  std::shared_ptr< project1::SearchNode > top = nullptr;

  // while the open_list isn't empty
  while( open_list.empty() == false ){
    //std::cout << "open_list.size():" << open_list.size() << " closed_list.size():" << closed_list.size() << std::endl;


    // get the node with smallest f 
    top = open_list.front();

    //std::cout << "top:" << top->x << "," << top->y << "," << top->f << "," << top->g << "," << top->h << std::endl;

    /*
    for( auto & open : open_list ){
      std::cout << "  open:" << open->x << "," << open->y << "," << open->theta << "," << open->f << "," << open->g << "," << open->h << std::endl;
    }

    for( auto & closed : closed_list ){
      std::cout << "  closed:" << closed->x << "," << closed->y << "," << closed->theta << "," << closed->f << "," << closed->g << "," << closed->h << std::endl;
    }

    std::cout << std::endl;

    if( closed_list.size() > 1000 ){
       exit(0);
    }
    */

    // check if the top node is the goal by checking if h==zero
    if(top->h <= 0.2){
      std::cout << "Goal is found!" << std::endl;
      
      // create a vector to store all the memory of the nodes in the path
      std::deque< std::shared_ptr< project1::SearchNode > > final_path;

      //create a nav_msgs::msg::Path message
      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = "map";
      //here many add the std_msg time_stamp

      //create a new deque for posestamped messages
      std::deque<geometry_msgs::msg::PoseStamped> pose_path;

      while(top != start_node->bp){
	// print out the final path in the terminal
	final_path.push_front(top);    
	std::cout << "top:" << *top << std::endl;
        
	//publish the path
	geometry_msgs::msg::PoseStamped pose_stamped;
	pose_stamped.header.frame_id = "map";
	pose_stamped.pose.position.x = top->x;
	pose_stamped.pose.position.y = top->y;
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
    
    // print top location and its memeory location
    // std::cout << "top:" << *top << std::endl;
    // std::cout << "top memory location:" << top << std::endl;

    // create descendents
    std::shared_ptr< project1::SearchNode > a = std::make_shared< project1::SearchNode>( top->x , top->y+0.2, M_PI/2 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< project1::SearchNode > b = std::make_shared< project1::SearchNode>( top->x , top->y-0.2, 3*M_PI/2 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< project1::SearchNode > c = std::make_shared< project1::SearchNode>( top->x+0.2, top->y, 0.0 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< project1::SearchNode > d = std::make_shared< project1::SearchNode>( top->x-0.2, top->y, M_PI , 0.0, 0.0, 0.0, top);
    std::shared_ptr< project1::SearchNode > m = std::make_shared< project1::SearchNode>( top->x+0.2, top->y+0.2, M_PI/4 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< project1::SearchNode > n = std::make_shared< project1::SearchNode>( top->x+0.2, top->y-0.2, -M_PI/4 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< project1::SearchNode > p = std::make_shared< project1::SearchNode>( top->x-0.2, top->y+0.2, 3*M_PI/4 , 0.0, 0.0, 0.0, top);
    std::shared_ptr< project1::SearchNode > q = std::make_shared< project1::SearchNode>( top->x-0.2, top->y-0.2, 5*M_PI/4 , 0.0, 0.0, 0.0, top);

    // compute the distance from descendents to the goal (h)
    a->h = (sqrt( std::pow( goal_node->x - a->x, 2.0 ) + std::pow( goal_node->y - a->y, 2.0 ) ) + fabs(goal_node->theta - a->theta));
    b->h = (sqrt( std::pow( goal_node->x - b->x, 2.0 ) + std::pow( goal_node->y - b->y, 2.0 ) ) + fabs(goal_node->theta - b->theta));
    c->h = (sqrt( std::pow( goal_node->x - c->x, 2.0 ) + std::pow( goal_node->y - c->y, 2.0 ) ) + fabs(goal_node->theta - c->theta));
    d->h = (sqrt( std::pow( goal_node->x - d->x, 2.0 ) + std::pow( goal_node->y - d->y, 2.0 ) ) + fabs(goal_node->theta - d->theta));
    m->h = (sqrt( std::pow( goal_node->x - m->x, 2.0 ) + std::pow( goal_node->y - m->y, 2.0 ) ) + fabs(goal_node->theta - m->theta));
    n->h = (sqrt( std::pow( goal_node->x - n->x, 2.0 ) + std::pow( goal_node->y - n->y, 2.0 ) ) + fabs(goal_node->theta - n->theta));
    p->h = (sqrt( std::pow( goal_node->x - p->x, 2.0 ) + std::pow( goal_node->y - p->y, 2.0 ) ) + fabs(goal_node->theta - p->theta));
    q->h = (sqrt( std::pow( goal_node->x - q->x, 2.0 ) + std::pow( goal_node->y - q->y, 2.0 ) ) + fabs(goal_node->theta - q->theta));
   
    // compute the distance from descendents to the start (g)
    a->g = 0.2 + abs(a->theta - a->bp->theta) + a->bp->g;
    b->g = 0.2 + abs(b->theta - b->bp->theta) + b->bp->g;
    c->g = 0.2 + abs(c->theta - c->bp->theta) + c->bp->g;
    d->g = 0.2 + abs(d->theta - d->bp->theta) + d->bp->g;
    m->g = 0.2 + abs(m->theta - m->bp->theta) + m->bp->g;
    n->g = 0.2 + abs(n->theta - n->bp->theta) + n->bp->g;
    p->g = 0.2 + abs(p->theta - p->bp->theta) + p->bp->g;
    q->g = 0.2 + abs(q->theta - q->bp->theta) + q->bp->g;
  
    // compute f by adding h and g for each descendent
    a->f = a->g +  a->h;
    b->f = b->g +  b->h;
    c->f = c->g +  c->h;
    d->f = d->g +  d->h;
    m->f = m->g +  m->h;
    n->f = n->g +  n->h;
    p->f = p->g +  p->h;
    q->f = q->g +  q->h;
    
    // create a vector to contain a,b,c,d
    std::vector<std::shared_ptr<project1::SearchNode>> children = {a,b,c,d,m,n,p,q};

    // check the four descendents
    for(auto& child:children){
       
      //convert (x,y) from meter unit to 1 per unit
      int col = static_cast<int>((child->x+25.6) / 0.2);
      int row = static_cast<int>((child->y+25.6) / 0.2);      

      // check if each descendent is out of the boundary
      if((col <0)||(col >= width)||(row <0)||(row >= height)){
        //closed_list.push_back(child);
        continue;
      }

      // check if each descendent is obstacle
      int map_index = twod_to_oned(col, row, width);
      if(expanded_map_data[map_index] == -128){
        //closed_list.push_back(child);
	continue;
      }

      // check if each descendent is in the closed_list	    
      bool in_closed_list=0;
      for(auto& closed_node:closed_list){
        if((fabs(child->x - closed_node->x)<0.1) && (fabs(child->y - closed_node->y)<0.1) && (fabs(child->theta - closed_node->theta)<0.2)){
	  in_closed_list=1;
	  break;
        }
      }

      bool in_open_list=0;
      for(auto& open_node:open_list){
        if((fabs(child->x - open_node->x)<0.1)  && (fabs(child->y - open_node->y)<0.1)  && (fabs(child->theta - open_node->theta)<0.2)){
          in_open_list=1;
          break;
        }
      }

      if((in_closed_list == 0)&&(in_open_list == 0)){
        open_list.push_back(child);
      }

    }

  // sort the open list
    std::sort( open_list.begin(), open_list.end(), []( const std::shared_ptr< project1::SearchNode >& firstArg, const std::shared_ptr< project1::SearchNode >& secondArg ) { return ( firstArg->f < secondArg->f ); } );
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
  int twod_to_oned(double x, double y, int w){
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

}

