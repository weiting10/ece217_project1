#include "project1/test_search_node.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <deque>
#include <algorithm>

namespace project1 {
  SearchNode::SearchNode( const int& xArg, const int& yArg , const double& thetaArg, const double& fArg, const double& gArg, const double& hArg , const std::shared_ptr< SearchNode >& bpArg ) : x(xArg), y(yArg), theta(thetaArg), f(fArg), g(gArg), h(hArg), bp(bpArg) {}

  SearchNode_rrt::SearchNode_rrt( const double& xArg, const double& yArg, const std::shared_ptr< SearchNode >& bpArg ) : x(xArg), y(yArg), bp(bpArg) {}
}
