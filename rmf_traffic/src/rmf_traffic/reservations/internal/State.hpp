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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATE_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATE_HPP

#include <rmf_traffic/reservations/Database.hpp>
#include <unordered_map>
#include <unordered_set>
#include <map>

#include "RequestQueue.hpp"

namespace rmf_traffic {
namespace reservations {

struct NextStateGenerator;
//TODO: Currrently uses a lot of copying. In future we should use a parent based
//hierarchy
class State
{
  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);
      return h1 ^ h2;
    }
  };
public:

  struct ReservationAssignmentIndex
  {
    ParticipantId participant;
    RequestId reqid;
    std::size_t index;
  };
  using ResourceSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ResourceSchedules = std::unordered_map<std::string, ResourceSchedule>;
  using ReservationAssignment = std::unordered_map<
    ParticipantId, std::unordered_map<ReservationId, RequestId>>;

  using ReservationTimings = std::unordered_map<ReservationId, Time>;
  using ReservationResources = std::unordered_map<ReservationId, std::string>;
  using ReservationRequestId = std::unordered_map<ReservationId,
    ReservationAssignmentIndex>;
  using UnassignedSet = std::unordered_set<
    std::pair<ParticipantId, RequestId>, pair_hash>;

  State(std::shared_ptr<RequestQueue> queue) :
    _queue(queue)
  {
    //Do nothing
  }

  NextStateGenerator begin()
  {
  }

  NextStateGenerator end()
  {
  }
  /// Adds a request to the state
  State add_request(
    ParticipantId pid,
    RequestId reqid) const
  {
    State new_state(*this);
    new_state._unassigned.insert({pid, reqid});
    return new_state;
  }

  State remove_request(
    ParticipantId pid,
    RequestId reqid) const
  {
    State new_state(*this);
    auto req = new_state._unassigned.find({pid, reqid});

    if(req != new_state._unassigned.end())
    {
      new_state._unassigned.erase(req);
      return new_state;
    }
    auto reservation_id = new_state._reservation_assignments[pid][reqid];
    auto findings = new_state._reservation_timings.find(reservation_id);
    if(findings == new_state._reservation_timings.end())
    {
      // Non existant pid-reqid pair
      return;
    }
    auto time = findings->second;
    auto resource = new_state._reservation_resources[reservation_id];

    new_state._resource_schedules[resource].erase(time);
    new_state._reservation_resources.erase(reservation_id);
    new_state._reservation_assignments.erase(reservation_id);
    new_state._reservation_timings.erase(findings);
    return new_state;
  }

  std::optional<State> delay_reservations_after(ReservationId id, Duration dur)
  {
    State new_state(*this);
    auto resource = new_state._reservation_resources.find(id);
    if(resource == new_state._reservation_resources.end())
    {
      // This is not possible
      return std::nullopt;
    }
    auto start_time = new_state._reservation_timings[id];
    auto res_iter = new_state._resource_schedules[resource->second]
      .find(start_time);
    auto prev_iter = res_iter;
    std::next(res_iter);

    // TODO: This is extremely inefficient
    std::map<Time, ReservationId> new_times;
    auto gap_left = dur;
    while(
      res_iter != new_state._resource_schedules[resource->second].end()
      && prev_iter->second.actual_finish_time().has_value()
      && gap_left.count() > 0)
    {
      auto new_time = res_iter->first + gap_left;
      auto gap = res_iter->first - prev_iter->second.actual_finish_time().value();
      if(gap < gap_left)
        gap_left -= gap;
      else
        gap_left -= gap_left;
      new_times[new_time] = res_iter->second.reservation_id();
      prev_iter = res_iter;
      std::next(res_iter);
    }

    for(auto iter = new_times.rbegin(); iter != new_times.rend(); iter++)
    {
      bool ok  = new_state.shift_reservation_start_time(
        resource->second,
        iter->second,
        iter->first);

      if(!ok) return std::nullopt;
    }

    return {new_state};
  }

  
  void purge_up_to_time(Time time)
  {

  }

  bool operator==(State& other) const
  {
    return _resource_schedules == other._resource_schedules
      && _reservation_assignments == other._reservation_assignments;
  }

  std::size_t hash()
  {

  }

  State(const State& other) :
    _resource_schedules(other._resource_schedules),
    _unassigned(other._unassigned),
    _reservation_timings(other._reservation_timings),
    _queue(other._queue),
    _reservation_resources(other._reservation_resources),
    _reservation_assignments(other._reservation_assignments),
    _reservation_request_ids(other._reservation_request_ids)
  {
    // Do nothing
  }

private:
  // Returns true if a shift could be successfully applied. Else returns false.
  void unassign_reservation(ResourceSchedule::const_iterator iter)
  {
    auto res_id = iter->second.reservation_id();
    auto resource_name = iter->second.resource_name();

    _reservation_timings.erase(res_id);
    _reservation_resources.erase(res_id);
    auto request = _reservation_request_ids[res_id];

    _unassigned.insert({request.participant , request.reqid});
    _reservation_assignments[request.participant].erase(request.reqid);
    _reservation_request_ids.erase(res_id);
  }

  bool shift_reservation_start_time(
    std::string resource,
    ReservationId res_id,
    rmf_traffic::Time new_time)
  {
    auto original_time = _reservation_timings[res_id];
    auto new_reservation =
      _resource_schedules[resource][original_time]
        .propose_new_start_time(new_time);
    auto request_info = _reservation_request_ids[res_id];
    auto index = _queue->satisfies(
      request_info.participant,
      request_info.reqid,
      new_reservation);
    if(!index.has_value())
    {
      return false;
    }
    _reservation_request_ids[res_id].reqid = *index;
    _resource_schedules[resource].erase(original_time);
    _resource_schedules[resource][new_time] = new_reservation;
    _reservation_timings[res_id] = new_time;
    return true;
  }

  ResourceSchedules _resource_schedules; // resource_name => Schedules
  ReservationAssignment _reservation_assignments; // {participant, req_id} => {reservation_id
  UnassignedSet _unassigned; // {participant, req_id}
  ReservationTimings _reservation_timings; // reservation => time
  ReservationResources _reservation_resources; // reservation => resource
  ReservationRequestId _reservation_request_ids; // reservation => {part_id, req_id, index}

  std::shared_ptr<RequestQueue> _queue;
  friend NextStateGenerator;
};
struct NextStateGenerator
{
  //TODO: RAW_POINTER-FOO
  State* start_state;
  std::optional<State> current_state;
  enum PotentialActionMode
  {
    ASSIGN_ITEMS,
    REMOVE_ITEMS,
    END
  };

  PotentialActionMode mode = ASSIGN_ITEMS;
  State::UnassignedSet::const_iterator unassigned_iter;
  ReservationRequest curr_request;
  State::ResourceSchedule::const_iterator insertion_point_iter;

  // For iterating through removals
  State::ResourceSchedules::const_iterator remove_resource_iter;
  State::ResourceSchedule::const_iterator remove_iter;
  std::size_t request_index = 0;

  using difference_type = std::ptrdiff_t;
  using value_type = State;
  using pointer = State*;
  using reference = State;
  using iterator_category= std::input_iterator_tag;

  State operator*() const {
    return *current_state;
  }

  State::ResourceSchedule::const_iterator
    get_search_start(ReservationRequest req)
  {
    if(req.start_time().has_value() &&
      req.start_time().value().lower_bound().has_value())
    {
      return start_state->
        _resource_schedules[req.resource_name()]
        .lower_bound(
          curr_request.start_time().value().lower_bound().value());
    }
    else
    {
      return start_state->_resource_schedules[req.resource_name()].begin();
    }
  }

  State::ResourceSchedule::const_iterator
    get_search_end(ReservationRequest req)
  {
    if(req.start_time().has_value() &&
      req.start_time().value().lower_bound().has_value())
    {
      return start_state->
        _resource_schedules[req.resource_name()]
        .upper_bound(
          curr_request.start_time().value().upper_bound().value());
    }
    else
    {
      return start_state->_resource_schedules[req.resource_name()].end();
    }
  }

  std::optional<State> advance_assignments()
  {
    
  }

  std::optional<State> advance_removals()
  {
    if(remove_resource_iter == start_state->_resource_schedules.end())
    {
      return std::nullopt;
    }
    while(remove_iter == remove_resource_iter->second.end())
    {
      remove_resource_iter++;
      remove_iter = remove_resource_iter->second.begin();
    }

    State new_state(start_state);
    new_state.remove_request(remove_iter);
    remove_iter++;
    return new_state;
  }

  std::optional<State> next_state() {
    if(mode == PotentialActionMode::ASSIGN_ITEMS)
    {
      if(unassigned_iter == start_state->_unassigned.end())
      {
        mode = PotentialActionMode::REMOVE_ITEMS;
      }
      else
      {
        auto [participant_id, request_id] = *unassigned_iter;
        auto info =
          start_state->_queue->get_request_info(participant_id, request_id);
        if(request_index >= info.request_options.size())
        {
          unassigned_iter++;
        }
        else
        {
          auto curr_request = info.request_options[request_index];
          // Get all reservations within current request and attempt insertion
          
          request_index++;
        }
      }
    }
    else if(mode == REMOVE_ITEMS)
    {

    }
  }

  NextStateGenerator& operator++() {
    current_state = next_state();
    return *this;
  }

  NextStateGenerator operator++(int) {
    NextStateGenerator r = *this;
    ++(*this);
    return r;
  }

  bool operator==(const NextStateGenerator& other)
  {
    if(mode == PotentialActionMode::END
    && other.mode == PotentialActionMode::END)
    {
      return true;
    }
    return false;
  }

  bool operator!=(const NextStateGenerator& other)
  {
    return !(*this == other);
  }
};

}
}
#endif