#ifndef PROJECT1_SEARCH_NODE
#define PROJECT1_SEARCH_NODE
#include <memory>
#include <iostream>
#include <vector>
#include <deque>


namespace project1 {
  int process_map(const std::vector<int>& map_data, int width, int height);
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
    out << "x:" << other.x << " y:" << other.y << "theta: " << other.theta <<  " f:" << other.f << " g:" << other.g << " h:"  << other.h << " bp:" << other.bp;
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


}


#endif
