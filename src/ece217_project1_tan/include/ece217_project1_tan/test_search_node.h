#ifndef PROJECT1_SEARCH_NODE
#define PROJECT1_SEARCH_NODE
#include <memory>
#include <iostream>
#include <vector>
#include <deque>


namespace ece217_project1_tan {
  int process_map(const std::vector<int>& map_data, int width, int height);
  int process_map_rrt(const std::vector<int>& map_data, int width, int height);

  class SearchNode {
  public:
    SearchNode( const int& xArg =0, const int& yArg = 0, const double& thetaArg = 0.0, const double& fArg = 0.0, const double& gArg = 0.0, const double& hArg = 0.0, const std::shared_ptr< SearchNode >& bpArg = nullptr);
    SearchNode( const SearchNode& other ) = default;
    
    int x;
    int y;

    double theta;
    double f;
    double g;
    double h;

    std::shared_ptr< SearchNode> bp;


  };

  inline std::ostream& operator<<(std::ostream& out, const SearchNode& other ){
    out << "x:" << other.x << " y:" << other.y << " theta: " << other.theta <<  " f:" << other.f << " g:" << other.g << " h:"  << other.h << " bp:" << other.bp;
    return out;
  }

  inline std::ostream& operator<<(std::ostream& out, const std::vector<std::shared_ptr< SearchNode > >& other ){
    out << "[" << other.size() << "]:{" << std::endl;
    for( auto & element : other ){
      out << "  " << *element << std::endl;
    }
    out << "}" << std::endl;
    return out;
  }
  

  inline std::ostream& operator<<(std::ostream& out, const std::deque<std::shared_ptr< SearchNode > >& other ){
    out << "[" << other.size() << "]:{" << std::endl;
    for( auto & element : other ){
      out << "  " << *element << std::endl;
    }

    out << "}" << std::endl;
    return out;
  }

  class SearchNode_rrt {
  public:
    SearchNode_rrt( const double& xArg =0.0, const double& yArg = 0.0, const std::shared_ptr< SearchNode_rrt >& bpArg = nullptr);
    SearchNode_rrt( const SearchNode_rrt& other ) = default;

    double x;
    double y;

    std::shared_ptr< SearchNode_rrt> bp;

  };

  inline std::ostream& operator<<(std::ostream& out, const SearchNode_rrt& other ){
    out << "x:" << other.x << " y:" << other.y << " bp:" << other.bp;
    return out;
  }

  inline std::ostream& operator<<(std::ostream& out, const std::vector<std::shared_ptr< SearchNode_rrt > >& other ){
    out << "[" << other.size() << "]:{" << std::endl;
    for( auto & element : other ){
      out << "  " << *element << std::endl;
    }
    out << "}" << std::endl;
    return out;
  }


  inline std::ostream& operator<<(std::ostream& out, const std::deque<std::shared_ptr< SearchNode_rrt > >& other ){
    out << "[" << other.size() << "]:{" << std::endl;
    for( auto & element : other ){
      out << "  " << *element << std::endl;
    }

    out << "}" << std::endl;
    return out;
  }




}


#endif
