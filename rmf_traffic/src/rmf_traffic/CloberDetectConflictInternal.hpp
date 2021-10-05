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

#ifndef SRC__RMF_UTILS__CLOBERDETECTCONFLICTINTERNAL_HPP
#define SRC__RMF_UTILS__CLOBERDETECTCONFLICTINTERNAL_HPP

#ifdef CLOBER_RMF
#include <rmf_traffic/CloberDetectConflict.hpp>

#include "geometry/ShapeInternal.hpp"

#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/Trajectory.hpp>

#include <unordered_map>

#include <rmf_traffic/agv/Graph.hpp>


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>


namespace rmf_traffic {

class CloberDetectConflict::Implementation
{
public:

  struct Conflict
  {
    Trajectory::const_iterator a_it;
    Trajectory::const_iterator b_it;
    Time time;
  };

  using Conflicts = std::vector<Conflict>;

  static CloberDetectConflict::ConflictNotice between(
    std::string name_a,
    const Trajectory& trajectory_a,
    Eigen::Vector2d pos_a,
    std::string name_b,
    const Trajectory& trajectory_b,
    Eigen::Vector2d pos_b,
    std::string graph_file);
};

/// The navigation graph for the robot
std::shared_ptr<const rmf_traffic::agv::Graph> graph;
std::map<const std::string, std::pair<Time, Eigen::Vector2d>> _fleet_pos;
std::map<const std::string, std::pair<std::size_t, Eigen::Vector2d>> _old_occupy;
std::map<std::string, const std::string> _map_occupy;
std::map<const std::string, Eigen::Vector3d> _old_traj;
} // namespace rmf_traffic

#endif

#endif // SRC__RMF_UTILS__CLOBERDETECTCONFLICTINTERNAL_HPP
