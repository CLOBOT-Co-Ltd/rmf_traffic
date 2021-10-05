/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef RMF_TRAFFIC__CLOBERDETECTCONFLICT_HPP
#define RMF_TRAFFIC__CLOBERDETECTCONFLICT_HPP

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Profile.hpp>
#include <exception>

#include <rmf_traffic/DetectConflict.hpp>

#include <rmf_traffic_msgs/msg/negotiation_notice.hpp>
#include <rmf_traffic_msgs/msg/clober_negotiation_notice.hpp>

#include <unordered_map>
#include <map>

namespace rmf_traffic {

//==============================================================================
class CloberDetectConflict
{
public:

  enum class Interpolate : uint16_t
  {
    CubicSpline
  };

  /// Checks if there are any conflicts between the two trajectories.
  ///
  /// \return true if a conflict exists between the trajectories, false
  /// otherwise.
  
  using ConflictNotice = rmf_traffic_msgs::msg::NegotiationNotice;
  
  static ConflictNotice between(
  std::string name_a,
  const Trajectory& trajectory_a,
  Eigen::Vector2d pos_a,
  std::string name_b,
  const Trajectory& trajectory_b,
  Eigen::Vector2d pos_b,
  std::string graph_file);
  
  class Implementation;
};

} // namespace rmf_traffic

#endif

#endif // RMF_TRAFFIC__CLOBERDETECTCONFLICT_HPP
