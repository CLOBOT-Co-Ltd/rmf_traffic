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

CloberDetectConflict::ConflictNotice CloberDetectConflict::between(
  std::string name_a,
  const Trajectory& trajectory_a,
  Eigen::Vector2d pos_a,
  std::string name_b,
  const Trajectory& trajectory_b,
  Eigen::Vector2d pos_b,
  std::string graph_file)
{
  return Implementation::between(
    name_a, trajectory_a, pos_a,
    name_b, trajectory_b, pos_b,
    graph_file);
}

namespace {

void check_arrive_goal(const Trajectory& trajectory, std::string name)
{
  if(trajectory.size() == 2)
  {
    if(trajectory[0].position() == trajectory[1].position())
    {
      if(_old_occupy.find(name) != _old_occupy.end()){
        const auto it = _old_occupy.find(name);
        _old_occupy.erase(it);
      }
    }
  }
}

void check_change_trajectory(const Trajectory& trajectory, std::string name)
{
  if(_old_traj.find(name) == _old_traj.end())
  {
    _old_traj.insert({name, trajectory[0].position()});
  } else {
    const auto it = _old_traj.find(name);
    if(it->second != trajectory[0].position())
    {
      if(_old_occupy.find(name) != _old_occupy.end()){
        const auto o_it = _old_occupy.find(name);
        Eigen::Vector2d old_pos = o_it->second.second.second;
        std::string old_pos_name = o_it->second.second.first;
        _old_occupy.erase(o_it);
        _old_occupy.insert({name, std::make_pair(0, std::make_pair(old_pos_name,old_pos))});
      }
      _old_traj.erase(it);
      _old_traj.insert({name, trajectory[0].position()});
    }
  }
}

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

std::pair<std::string, Eigen::Vector2d> set_occupy(std::pair<std::vector<std::string>, Trajectory> traj, Eigen::Vector2d pos, std::string name)
{
  Trajectory::const_iterator it = traj.second.begin();
  std::size_t index = 0;

  // 로봇이 waypoint 위에 있는 경우 -> 다음 waypoint 반환
  while(it != traj.second.end())
  {
    const Trajectory::Waypoint& way = *(it);

    if(abs(way.position().x() - pos.x()) < 0.3 && abs(way.position().y() - pos.y()) < 0.3)
    {
      Trajectory::const_iterator it_buff = it;

      if(++it_buff != traj.second.end())
      {
        ++it; 
      }
      const Trajectory::Waypoint& occupy = *(it);

      if(_fleet_pos.find(name) != _fleet_pos.end()){
        const auto f_it = _fleet_pos.find(name);
        _fleet_pos.erase(f_it);
      }

      if(index < traj.first.size() - 1) index++;

      _fleet_pos.insert({name, std::make_pair(traj.first[index], Eigen::Vector2d(occupy.position().x(), occupy.position().y()))});
      return std::make_pair(traj.first[index], Eigen::Vector2d(occupy.position().x(), occupy.position().y()));
    }
    ++it;
    ++index;
  }

  // 로봇이 waypoint에 있지 않는 경우 -> 이전의 occupy set 반환
  if(_fleet_pos.find(name) != _fleet_pos.end())
  {
    const auto f_it = _fleet_pos.find(name);
    return std::make_pair(f_it->second.first, f_it->second.second);
  } else {  // 이전의 occupy_set이 없는 경우 -> 가장 가까운 waypoint 점유
    it = traj.second.begin();
    Trajectory::const_iterator it_min = it;
    index = 0;
    std::size_t min_index = 0;
    double diff_min = 100.0;

    while(it != traj.second.end())
    {
      const Trajectory::Waypoint& way = *(it);
      double diff_buff = pow(way.position().x() - pos.x(), 2) + pow(way.position().y() - pos.y(), 2);

      if(diff_buff < diff_min)
      {
        diff_min = diff_buff;
        min_index = index;
        it_min = it;
      }
      ++index;
      ++it;
    }

    const Trajectory::Waypoint& occupy = *(it_min);

    _fleet_pos.insert({name, std::make_pair(traj.first[min_index], Eigen::Vector2d(occupy.position().x(), occupy.position().y()))});
    return std::make_pair(traj.first[min_index], Eigen::Vector2d(occupy.position().x(), occupy.position().y()));
  }
}

std::size_t set_occupy_idx(std::pair<std::string, Eigen::Vector2d> occupy, std::string name)
{
  if(_old_occupy.find(name) == _old_occupy.end())
  {
    _old_occupy.insert({name, std::make_pair(0, occupy)});
  }

  std::string pos = occupy.first;
  if(_map_occupy.find(pos) == _map_occupy.end())
  {
    _map_occupy.insert({pos, name});
  }

  const auto o_it = _old_occupy.find(name);

  if((occupy.second.x() != o_it->second.second.second.x()) || occupy.second.y() != o_it->second.second.second.y())
  {
    int idx = o_it->second.first + 1;
    std::string old_pos = o_it->second.second.first;
    _old_occupy.erase(o_it);
    _old_occupy.insert({name, std::make_pair(idx, occupy)});
    

    if(_map_occupy.find(old_pos) != _map_occupy.end()){
      const auto m_it = _map_occupy.find(old_pos);
      if(m_it->second == name) _map_occupy.erase(m_it);
    }

    std::cout << "======map occupy set======" << std::endl;
    std::cout << "robot name : occupy axis" << std::endl;
    for (const auto& map_occupy : _map_occupy)
    {
      const auto it = _old_map_occupy.find(map_occupy.first);
      if(it->second == map_occupy.second) std::cout << map_occupy.second << " : " << map_occupy.first << std::endl;
      else std::cout << "\x1b[33m" << map_occupy.second << " : " << map_occupy.first << "\x1b[37m" << std::endl;
    }

    _old_map_occupy = _map_occupy;

    return idx;
  } else {
    return o_it->second.first;
  }
}

}

CloberDetectConflict::ConflictNotice CloberDetectConflict::Implementation::between(
  std::string name_a,
  const Trajectory& trajectory_a,
  Eigen::Vector2d pos_a,
  std::string name_b,
  const Trajectory& trajectory_b,
  Eigen::Vector2d pos_b,
  std::string graph_file)
{
  CloberDetectConflict::ConflictNotice msg;

  check_arrive_goal(trajectory_a, name_a);
  check_arrive_goal(trajectory_b, name_b);

  check_change_trajectory(trajectory_a, name_a);
  check_change_trajectory(trajectory_b, name_b);

  graph = std::make_shared<rmf_traffic::agv::Graph>(parse_graph(graph_file));

  std::pair<std::vector<std::string>, Trajectory> traj_a = compare_graph(trajectory_a);
  std::pair<std::vector<std::string>, Trajectory> traj_b = compare_graph(trajectory_b);

  /* 점유 설정 */
  std::pair<std::string, Eigen::Vector2d> occupy_a = set_occupy(traj_a, pos_a, name_a);
  std::pair<std::string, Eigen::Vector2d> occupy_b = set_occupy(traj_b, pos_b, name_b);

  std::size_t idx_a = set_occupy_idx(occupy_a, name_a);
  std::size_t idx_b = set_occupy_idx(occupy_b, name_b);

  /* 충돌 체크 */
  if(occupy_a.first == occupy_b.first)
  {
    rmf_traffic_msgs::msg::CloberNegotiationNotice CNN;

    std::string pos = occupy_a.first;

    const auto it = _map_occupy.find(pos);
    if(it->second == name_b) {
      std::cout << "\x1b[31m" << name_a << " : " << occupy_a.first << "\x1b[37m" << std::endl;
      //target
      CNN.robotid = name_a;
      if(idx_a == 0) CNN.startidx = 0;
      else CNN.startidx = idx_a - 1;
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

      msg.robot_info.push_back(CNN);

      //enemy
      CNN.robotid = name_b;
      if(idx_b == 0) CNN.startidx = 0;
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

      const auto it_a = _old_occupy.find(name_a);
      _old_occupy.erase(it_a);
      _old_occupy.insert({name_a, std::make_pair(1, occupy_a)});
      const auto it_b = _old_occupy.find(name_b);
      _old_occupy.erase(it_b);
      _old_occupy.insert({name_b, std::make_pair(0, occupy_b)});

      return msg;
    }
    else if(it->second == name_a) {
      std::cout << "\x1b[31m" << name_b << " : " << occupy_a.first << "\x1b[37m" << std::endl;
      //target
      CNN.robotid = name_b;
      if(idx_b == 0) CNN.startidx = 0;
      else CNN.startidx = idx_b - 1;
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

      //enemy
      CNN.robotid = name_a;
      if(idx_a == 0) CNN.startidx = 0;
      CNN.startidx = idx_a;
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

      msg.robot_info.push_back(CNN);

      const auto it_a = _old_occupy.find(name_b);
      _old_occupy.erase(it_a);
      _old_occupy.insert({name_b, std::make_pair(1, occupy_b)});
      const auto it_b = _old_occupy.find(name_a);
      _old_occupy.erase(it_b);
      _old_occupy.insert({name_a, std::make_pair(0, occupy_a)});

      return msg;
    } else {
      std::cout << "\x1b[34m" << "충돌 체크했으나, 점유 우선권 확인 불가" << "\x1b[37m" << std::endl;
      return msg;
    }
  }  

  /* 충돌 없음 */
  return msg;
}


}

#endif