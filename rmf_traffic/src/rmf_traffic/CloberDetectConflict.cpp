/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifdef CLOBER_RMF

#include "geometry/ShapeInternal.hpp"
#include "CloberDetectConflictInternal.hpp"
#include "ProfileInternal.hpp"
#include "Spline.hpp"
#include "StaticMotion.hpp"

#ifdef RMF_TRAFFIC__USING_FCL_0_6
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/math/motion/spline_motion.h>
#include <fcl/narrowphase/collision.h>
#else
#include <fcl/continuous_collision.h>
#include <fcl/ccd/motion.h>
#include <fcl/collision.h>
#endif

#include <unordered_map>

#include <rmf_traffic/geometry/Circle.hpp>
#include <yaml-cpp/yaml.h>

namespace rmf_traffic {

//==============================================================================
rmf_utils::optional<rmf_traffic::Time> CloberDetectConflict::between(
  std::string name_a,
  const Trajectory& trajectory_a,
  Eigen::Vector2d pos_a,
  std::string name_b,
  const Trajectory& trajectory_b,
  Eigen::Vector2d pos_b)
{
  return Implementation::between(
    name_a, trajectory_a, pos_a,
    name_b, trajectory_b, pos_b);
}

CloberDetectConflict::ConflictNotice CloberDetectConflict::between2(
  std::string name_a,
  const Trajectory& trajectory_a,
  Eigen::Vector2d pos_a,
  std::string name_b,
  const Trajectory& trajectory_b,
  Eigen::Vector2d pos_b,
  std::string graph_file)
{
  return Implementation::between2(
    name_a, trajectory_a, pos_a,
    name_b, trajectory_b, pos_b,
    graph_file);
}

namespace {

rmf_traffic::agv::Graph parse_graph(const std::string& graph_file)
{
  const YAML::Node graph_config = YAML::LoadFile(graph_file);
  if (!graph_config)
  {
    throw std::runtime_error("Failed to load graph file [" + graph_file + "]");
  }

  const YAML::Node levels = graph_config["levels"];

  rmf_traffic::agv::Graph graph;

  for (const auto& level : levels)
  {
    const std::string& map_name = level.first.as<std::string>();

    const YAML::Node& vertices = level.second["vertices"];
    for (const auto& vertex : vertices)
    {
      const Eigen::Vector2d location{
        vertex[0].as<double>(), vertex[1].as<double>()};

      auto& wp = graph.add_waypoint(map_name, location);

      const YAML::Node& options = vertex[2];
      const YAML::Node& name_option = options["name"];
      if (name_option)
      {
        const std::string& name = name_option.as<std::string>();
        if (!name.empty())
        {
          if (!graph.add_key(name, wp.index()))
          {
            // *INDENT-OFF*
            throw std::runtime_error(
              "Duplicated waypoint name [" + name + "] in graph ["
              + graph_file + "]");
            // *INDENT-ON*
          }
        }
      }
    }
  }

  return graph;
}

std::pair<std::vector<std::string>, Trajectory> compare_graph(const Trajectory& trajectory)
{
  Trajectory traj;
  std::vector<std::string> name;

  const std::string* node_name = nullptr;
  const std::string* old_node_name = nullptr;

  for(std::size_t i = 0; i < trajectory.size(); i++)
  {
    double pos_x = trajectory[i].position().x();
    double pos_y = trajectory[i].position().y();

    for(std::size_t j = 0; j < graph->num_waypoints(); j++)
    {
      node_name = graph->get_waypoint(j).name();
      double graph_x = graph->get_waypoint(j).get_location().x();
      double graph_y = graph->get_waypoint(j).get_location().y();

      double diff_x = abs(pos_x - graph_x);
      double diff_y = abs(pos_y - graph_y);

      if(diff_x < 0.1 && diff_y < 0.1)
      {
        if(node_name == old_node_name)
          continue;

        traj.insert(trajectory[i]);
        name.push_back(*node_name);
        old_node_name = graph->get_waypoint(j).name();
        break;
      }
    }
  }

  if(traj.size() < 2)
    traj = trajectory;

  return std::make_pair(name, traj);
}

std::pair<Time, Eigen::Vector2d> set_occupy(const Trajectory traj, Eigen::Vector2d pos, std::string name)
{
  Trajectory::const_iterator it = traj.begin();

  // 로봇이 waypoint 위에 있는 경우 -> 다음 waypoint 반환
  // std::cout << " set_occupy1" << std:: endl;
  while(it != traj.end())
  {
    const Trajectory::Waypoint& way = *(it);
    // std::cout << "way : " << way.position().x() << " , " << way.position().y() << std::endl;
    // std::cout << "pos : " << pos.x() << " , " << pos.y() << std::endl;

    if(abs(way.position().x() - pos.x()) < 0.1 && abs(way.position().y() - pos.y()) < 0.1)
    {
      Trajectory::const_iterator it_buff = it;

      if(++it_buff != traj.end()) ++it; 

      const Trajectory::Waypoint& occupy = *(it);

      if(_fleet_pos.find(name) != _fleet_pos.end()){
        const auto f_it = _fleet_pos.find(name);
        _fleet_pos.erase(f_it);
      }

      _fleet_pos.insert({name, std::make_pair(occupy.time(), Eigen::Vector2d(occupy.position().x(), occupy.position().y()))});
      return std::make_pair(occupy.time(), Eigen::Vector2d(occupy.position().x(), occupy.position().y()));
    }
    ++it;
  }

  // 로봇이 waypoint에 있지 않는 경우 -> 이전의 occupy set 반환
  if(_fleet_pos.find(name) != _fleet_pos.end())
  {
    // std::cout << " set_occupy2" << std:: endl;

    const auto f_it = _fleet_pos.find(name);
    return std::make_pair(f_it->second.first, f_it->second.second);
  } else {  // 이전의 occupy_set이 없는 경우 -> 가장 가까운 waypoint 점유
    // std::cout << " set_occupy3" << std:: endl;

    it = traj.begin();
    Trajectory::const_iterator it_min = it;
    double diff_min = 100.0;

    while(it != traj.end())
    {
      const Trajectory::Waypoint& way = *(it);
      double diff_buff = pow(way.position().x() - pos.x(), 2) + pow(way.position().y() - pos.y(), 2);

      if(diff_buff < diff_min)
      {
        diff_min = diff_buff;
        it_min = it;
      }

      ++it;
    }

    const Trajectory::Waypoint& occupy = *(it_min);

    _fleet_pos.insert({name, std::make_pair(occupy.time(), Eigen::Vector2d(occupy.position().x(), occupy.position().y()))});
    return std::make_pair(occupy.time(), Eigen::Vector2d(occupy.position().x(), occupy.position().y()));
  }
}

std::size_t check_start_idx(Eigen::Vector2d occupy_pos, std::vector<std::string> path)
{
  for(std::size_t i = 0; i < graph->num_waypoints(); i++)
  {
    // std::cout << graph->get_waypoint(i).get_location().x() << " , " << graph->get_waypoint(i).get_location().y() << std:: endl;
    if(abs(occupy_pos.x() - graph->get_waypoint(i).get_location().x()) < 0.1
    && abs(occupy_pos.y() - graph->get_waypoint(i).get_location().y()) < 0.1)
    {
      for(std::size_t j = 0; j < path.size(); j++)
      {
        if(path[j] == *graph->get_waypoint(i).name())
          return j;
      }
    }
  }
}


}

//==============================================================================
rmf_utils::optional<rmf_traffic::Time> CloberDetectConflict::Implementation::between(
  std::string name_a,
  const Trajectory& trajectory_a,
  Eigen::Vector2d pos_a,
  std::string name_b,
  const Trajectory& trajectory_b,
  Eigen::Vector2d pos_b)
{
  const std::string graph_file =
  "/home/clober/clober_rmf_ws/install/clober_rmf/share/clober_rmf/3x3/nav_graphs.yaml";

  graph = std::make_shared<rmf_traffic::agv::Graph>(
    parse_graph(graph_file));

  std::pair<std::vector<std::string>, Trajectory> traj_a = compare_graph(trajectory_a);
  std::pair<std::vector<std::string>, Trajectory> traj_b = compare_graph(trajectory_b);

  /* 점유 설정 */
  std::pair<Time, Eigen::Vector2d> occupy_a = set_occupy(traj_a.second, pos_a, name_a);
  std::pair<Time, Eigen::Vector2d> occupy_b = set_occupy(traj_b.second, pos_b, name_b);

  /* 충돌 체크 */
  if(abs(occupy_a.second.x() - occupy_b.second.x()) < 0.1 && abs(occupy_a.second.y() - occupy_b.second.y()) < 0.1)
  {
    std::cout << "Occur Collision" << std::endl;
    std::cout << "Collision Check for " << name_a << " , " << name_b << std::endl;
    return std::min(occupy_a.first, occupy_b.first);
  }  

  /* 충돌 없음 */
  // std::cout << "No Collision" << std::endl;
  return rmf_utils::nullopt;
}

CloberDetectConflict::ConflictNotice CloberDetectConflict::Implementation::between2(
  std::string name_a,
  const Trajectory& trajectory_a,
  Eigen::Vector2d pos_a,
  std::string name_b,
  const Trajectory& trajectory_b,
  Eigen::Vector2d pos_b,
  std::string graph_file)
{
  CloberDetectConflict::ConflictNotice msg;

  // const std::string graph_file =
  // "/home/clober/clober_rmf_ws/install/clober_rmf/share/clober_rmf/3x3/nav_graphs.yaml";

  graph = std::make_shared<rmf_traffic::agv::Graph>(
    parse_graph(graph_file));

  std::pair<std::vector<std::string>, Trajectory> traj_a = compare_graph(trajectory_a);
  std::pair<std::vector<std::string>, Trajectory> traj_b = compare_graph(trajectory_b);

  /* 점유 설정 */
  std::pair<Time, Eigen::Vector2d> occupy_a = set_occupy(traj_a.second, pos_a, name_a);
  std::pair<Time, Eigen::Vector2d> occupy_b = set_occupy(traj_b.second, pos_b, name_b);

  // std::cout << "pos_a: " << pos_a.x() << " , " << pos_a.y() << std::endl;
  // std::cout << "occupy_a: " << occupy_a.second.x() << " , " << occupy_a.second.y() << std::endl;
  // std::cout << "pos_b: " << pos_b.x() << " , " << pos_b.y() << std::endl;
  // std::cout << "occupy_b: " << occupy_b.second.x() << " , " << occupy_b.second.y() << std::endl;

  /* 충돌 체크 */
  if(abs(occupy_a.second.x() - occupy_b.second.x()) < 0.1 && abs(occupy_a.second.y() - occupy_b.second.y()) < 0.1)
  {
    std::cout << "Occur Collision" << std::endl;
    std::cout << "Collision Check for " << name_a << " , " << name_b << std::endl;
   
    rmf_traffic_msgs::msg::CloberNegotiationNotice CNN;

    //target
    CNN.robotid = name_a;
    int idx_a = check_start_idx(occupy_a.second, traj_a.first) - 1;
    if(idx_a < 0) CNN.startidx = 0;
    else CNN.startidx = idx_a;
    std::vector<std::string> path_a;
    if(traj_a.first.size() == 0) {
      path_a.push_back(traj_a.first[0]);
    } else {
      for(std::size_t i = CNN.startidx; i < traj_a.first.size(); i++)
      {
        path_a.push_back(traj_a.first[i]);
      }
    }
    CNN.startidx = 0;
    CNN.path = path_a;
    CNN.start = path_a.front();
    CNN.end = traj_a.first.back();
    // CNN.path = traj_a.first;
    // CNN.start = traj_a.first.front();
    // CNN.startidx = check_start_idx(occupy_a.second, traj_a.first);

    msg.robot_info.push_back(CNN);

    //enemy
    // CNN.robotid = name_b;
    // CNN.path = traj_b.first;
    // CNN.start = traj_b.first.front();
    // int idx_b = check_start_idx(occupy_b.second, traj_b.first) - 1;
    // if(idx_b < 0) CNN.startidx = 0;
    // else CNN.startidx = idx_b;
    // CNN.end = traj_b.first.back();
    CNN.robotid = name_b;
    int idx_b = check_start_idx(occupy_b.second, traj_b.first) - 1;
    if(idx_b < 0) CNN.startidx = 0;
    else CNN.startidx = idx_b;
    std::vector<std::string> path_b;
    if(traj_b.first.size() == 0) {
      path_b.push_back(traj_b.first[0]);
    } else {
      for(std::size_t i = CNN.startidx; i < traj_b.first.size(); i++)
      {
        path_b.push_back(traj_b.first[i]);
      }
    }
    CNN.startidx = 0;
    CNN.path = path_b;
    CNN.start = path_b.front();
    CNN.end = traj_b.first.back();

    msg.robot_info.push_back(CNN);

    return msg;
  }  

  /* 충돌 없음 */
  // std::cout << "No Collision" << std::endl;
  return msg;
}


}

#endif