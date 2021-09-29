#ifdef CLOBER_RMF

#include "CloberBFS.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

CBFS::CBFS(std::map<std::string, std::vector<std::string> > graph) : graph_(graph) {
    // std::cout << "bfs instance" << std::endl;
}

CBFS::~CBFS() {
}

void CBFS::printPath() {
    for (int i = 0; i < path_.size(); i++) {
        std::cout << " [ ";
        for (int j = 0; j < path_[i].size(); j++) {
            std::cout << "n" << stoi(path_[i][j]) + 1;
            if (j < path_[i].size() - 1)
                std::cout << " , ";
        }
        std::cout << " ] " << std::endl;
    }
}

void CBFS::printGraph() {
    std::map<std::string, std::vector<std::string> >::iterator it;
    for (it = graph_.begin(); it != graph_.end(); it++) {
        std::cout << "key : " << it->first << " value : ";
        for (int i = 0; i < it->second.size(); i++) {
            std::cout << it->second[i] << " ";
        }
        std::cout << "\n"
                  << std::endl;
    }
}

void CBFS::bfs_paths(std::string start, std::string goal, std::vector<std::vector<std::string> > &nodePath) {
    path_.clear();
    path_.resize(0);
    
    std::vector<std::pair<std::string, std::vector<std::string> > > queue;
    std::vector<std::string> startVec;
    startVec.push_back(start);
    queue.push_back(std::make_pair(start, startVec));

    std::vector<std::vector<std::string> > result;

    while (!queue.empty()) {
        std::string n;
        std::vector<std::string> path;

        n = queue.at(0).first;
        path = queue.at(0).second;

        queue.erase(queue.begin());

        if (n == goal) {
            result.push_back(path);
            break;
        } else {
            std::vector<std::string> graph_n;
            graph_n = graph_.find(n)->second;

            std::vector<std::string> set;

            for (int i = 0; i < graph_n.size(); i++) {
                std::string val = graph_n[i];

                auto it = std::find(path.begin(), path.end(), val);
                if (it == path.end()) {
                    set.push_back(val);
                }
            }

            // set.resize(graph_n.size() + path.size());

            // std::vector<std::string>::iterator itr;
            // itr = std::set_difference(graph_n.begin(), graph_n.end(), path.begin(), path.end(), set.begin());
            // set.erase(itr, set.end());

            // std::set_difference(graph_n.begin(), graph_n.end(), path.begin(), path.end(), std::back_inserter(set));

            for (int i = 0; i < set.size(); i++) {
                std::vector<std::string> ext = path;
                ext.push_back(set[i]);
                // path.push_back(set[i]);
                queue.push_back(std::make_pair(set[i], ext));
            }
        }
    }

    path_ = result;
    nodePath.resize(path_.size());
    nodePath.assign(path_.begin(), path_.end());    
}

}  // namespace planning
}  // namespace agv
}  // namespace rmf_traffic

#endif