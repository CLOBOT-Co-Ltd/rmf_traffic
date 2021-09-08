

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__CLOBERBFS_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__CLOBERBFS_HPP

#ifdef CLOBER_RMF

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace rmf_traffic {
namespace agv {
namespace planning {

class CBFS {
   public:
    CBFS(std::map<std::string, std::vector<std::string> > graph);
    ~CBFS();

    void bfs_paths(std::string start, std::string goal, std::vector<std::vector<std::string> > &nodePath);
    void printPath();
    void printGraph();

   private:
    std::map<std::string, std::vector<std::string> > graph_;
    std::vector<std::vector<std::string> > path_;
};
}
}
}

#endif

#endif  //SRC__RMF_TRAFFIC__AGV__PLANNING__CLOBERBFS_HPP