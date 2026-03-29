#include <iostream>
#include <math.h>
#include "project1/test_search_node.h"
#include <vector>
#include <deque>
#include <algorithm>

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

  std::cout << "open_list" << open_list << std::endl;
  std::cout << "closed_list" << closed_list << std::endl;

  // creat shared pointer top
  std::shared_ptr< project1::SearchNode > top = nullptr;

  // while the open_list isn't empty
  while( open_list.empty() == false ){
    
    // get the node with smallest f 
    top = open_list.front();

    // check if the top node is the goal by checking if h==zero
    if(top->h <= 0.1){
      std::cout << "Goal is found!" << std::endl;
      
      // create a vector to store all the memory of the nodes in the path
      std::deque< std::shared_ptr< project1::SearchNode > > final_path;

      while(top != start_node->bp){
	final_path.push_front(top);    
	std::cout << "top:" << *top << std::endl;
        top=top->bp;
      }
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

    // compute the distance from descendents to the goal (h)
    a->h = sqrt( std::pow( goal_node->x - a->x, 2.0 ) + std::pow( goal_node->y - a->y, 2.0 ) ) ;
    b->h = sqrt( std::pow( goal_node->x - b->x, 2.0 ) + std::pow( goal_node->y - b->y, 2.0 ) ) ;
    c->h = sqrt( std::pow( goal_node->x - c->x, 2.0 ) + std::pow( goal_node->y - c->y, 2.0 ) ) ;
    d->h = sqrt( std::pow( goal_node->x - d->x, 2.0 ) + std::pow( goal_node->y - d->y, 2.0 ) ) ;
  
    // compute the distance from descendents to the start (g)
    a->g = 0.2 + abs(a->theta - a->bp->theta) + a->bp->g;
    b->g = 0.2 + abs(b->theta - b->bp->theta) + b->bp->g;
    c->g = 0.2 + abs(c->theta - c->bp->theta) + c->bp->g;
    d->g = 0.2 + abs(d->theta - d->bp->theta) + d->bp->g;

    // compute f by adding h and g for each descendent
    a->f = a->g + a->h;
    b->f = b->g + b->h;
    c->f = c->g + c->h;
    d->f = d->g + d->h;
  
    // create a vector to contain a,b,c,d
    std::vector<std::shared_ptr<project1::SearchNode>> children = {a,b,c,d};

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
        if(fabs(child->x - closed_node->x)<1e-5 && fabs(child->y - closed_node->y)<1e-5 && fabs(child->theta - closed_node->theta)<1e-5){
	  in_closed_list=1;
	  break;
        }
      }

      bool in_open_list=0;
      for(auto& open_node:open_list){
        if(fabs(child->x - open_node->x)<1e-5 && fabs(child->y - open_node->y)<1e-5 && fabs(child->theta - open_node->theta)<1e-5){
          in_open_list=1;
          break;
        }
      }

	
	bool in_closed_list = (closed_set.count(key) > 0);
	bool in_open_list   = (open_set.count(key)   > 0);

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

