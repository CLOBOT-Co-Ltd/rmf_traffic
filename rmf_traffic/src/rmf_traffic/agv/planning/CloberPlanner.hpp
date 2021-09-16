#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__CLOBERPLANNER_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__CLOBERPLANNER_HPP

#ifdef CLOBER_RMF

#include "../internal_planning.hpp"
#include "DifferentialDriveHeuristic.hpp"


#include "CloberPlanner.hpp"
#include "CloberBFS.hpp"
#include "CloberMIP.hpp"


#include <iostream>
#include <cmath>


namespace rmf_traffic {
namespace agv {
namespace planning {

struct RobotInfo{
     std::string robotId_;
     std::vector<std::string> path_;
     std::string start_;
     std::size_t startIdx_;
     std::string end_;
};

class CloberPlanner : public Interface{

public:
    CloberPlanner(Planner::Configuration config);

    State initiate(
        const std::vector<agv::Planner::Start> &starts,
        agv::Planner::Goal goal,
        agv::Planner::Options options) const final;

    std::optional<PlanData> plan(State& state) const final;

    std::optional<PlanData> clober_plan(State& state, 
    std::string target_robot_id,
    std::string target_start,
    std::string target_end,
    std::vector<std::string> target_path,
    std::string enemy_robot_id,
    std::string enemy_start,
    std::size_t enemy_startidx,
    std::string enemy_end,
    std::vector<std::string> enemy_path) const final;

    std::vector<schedule::Itinerary> rollout(
    const Duration span,
    const Issues::BlockedNodes& nodes,
    const Planner::Goal& goal,
    const Planner::Options& options,
    std::optional<std::size_t> max_rollouts) const final;

  const Planner::Configuration& get_configuration() const final;

    std::unique_ptr<Debugger> debug_begin(
        const std::vector<Planner::Start>& starts,
        Planner::Goal goal,
        Planner::Options options) const final;

  std::optional<PlanData> debug_step(Debugger& debugger) const final;

  void GenerateGraph();
  void testBFS(std::string s, std::string g);
  void testMIP();


  std::vector<std::string> FindNewPath(RobotInfo target, RobotInfo enemy) const;
  bool convertNametoIdPath(const std::vector<std::string> path, std::vector<std::size_t> &idpath) const;

  void GenerateMIPValues();



private:
    Planner::Configuration _config;
    std::shared_ptr<const Supergraph> _supergraph;
    CacheManagerPtr<DifferentialDriveHeuristic> _cache;

    std::shared_ptr<CBFS> pbfs_;
    std::map<std::string, std::vector<std::string> > bfsGraph_;

    std::vector<std::string> path_;  
    std::vector<std::size_t> pathById_;  

    std::shared_ptr<COrtools> scheduler_;

    std::map<std::string, std::size_t> name_idGraph_;
    std::map<std::size_t, std::string> id_nameGraph_;

    mutable std::mutex plan_mutex_;

};




}
}
}

#endif

#endif //SRC__RMF_TRAFFIC__AGV__PLANNING__CLOBERPLANNER_HPP