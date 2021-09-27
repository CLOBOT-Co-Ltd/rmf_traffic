#ifdef CLOBER_RMF

#include "CloberPlanner.hpp"

#include "../internal_Planner.hpp"


#include "a_star.hpp"

#include <rmf_utils/math.hpp>
#include <set>

#include <rclcpp/node.hpp>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
template<typename NodePtr>
std::vector<NodePtr> reconstruct_nodes(const NodePtr& finish_node)
{
  NodePtr node = finish_node;
  std::vector<NodePtr> node_sequence;
  while (node)
  {
    node_sequence.push_back(node);
    node = node->parent;
  }

  return node_sequence;
}

namespace {
//==============================================================================
template<typename NodePtr>
void reparent_node_for_holding(
  const NodePtr& low_node,
  const NodePtr& high_node,
  Route route)
{
  // TODO(MXG): Rework this implementation so that the pointed-to types can
  // be const-qualified.
  high_node->parent = low_node;
  high_node->route_from_parent = {std::move(route)};
}

//==============================================================================
template<typename NodePtr>
struct OrientationTimeMap
{
  struct Element
  {
    using TimeMap = std::map<rmf_traffic::Time, NodePtr>;
    using TimePair = std::pair<
      typename TimeMap::iterator,
      typename TimeMap::iterator
    >;

    double yaw;
    TimeMap time_map;

    void squash(const agv::RouteValidator* validator)
    {
      assert(!time_map.empty());
      if (time_map.size() <= 2)
        return;

      std::vector<TimePair> queue;
      queue.push_back({time_map.begin(), --time_map.end()});

      while (!queue.empty())
      {
        const auto top = queue.back();
        queue.pop_back();
        const auto it_low = top.first;
        const auto it_high = top.second;

        assert(it_low != time_map.end());
        assert(it_high != time_map.end());

        if (it_high->first <= it_low->first)
          continue;

        if (it_low == --typename TimeMap::iterator(it_high))
        {
          // If the iterators are perfectly next to each other, then the only
          // action between them must be a holding.
          continue;
        }

        assert(it_high != time_map.begin());

        const auto& node_low = it_low->second;
        const auto& node_high = it_high->second;
        assert(node_low->route_from_parent.back().map()
          == node_high->route_from_parent.back().map());

        const auto& start_wp =
          node_low->route_from_parent.back().trajectory().back();

        const auto& end_wp =
          node_high->route_from_parent.back().trajectory().back();

        Route new_route{node_low->route_from_parent.back().map(), {}};
        new_route.trajectory().insert(start_wp);
        new_route.trajectory().insert(
          end_wp.time(),
          end_wp.position(),
          Eigen::Vector3d::Zero());

        if (!validator || !validator->find_conflict(new_route))
        {
          reparent_node_for_holding(node_low, node_high, std::move(new_route));
          time_map.erase(++typename TimeMap::iterator(it_low), it_high);
          queue.clear();
          if (time_map.size() > 2)
            queue.push_back({time_map.begin(), --time_map.end()});

          continue;
        }

        const auto next_high = --typename TimeMap::iterator(it_high);
        const auto next_low = ++typename TimeMap::iterator(it_low);

        if (next_high != it_low)
          queue.push_back({it_low, next_high});

        if (next_low != it_high)
          queue.push_back({next_low, it_high});
      }
    }
  };

  void insert(NodePtr node)
  {
    const auto yaw = node->yaw;
    const auto time =
      *node->route_from_parent.back().trajectory().finish_time();

    auto it = elements.begin();
    for (; it != elements.end(); ++it)
    {
      // TODO(MXG): Make this condition configurable
      if (std::abs(it->yaw - yaw) < 15.0*M_PI/180.0)
        break;
    }

    if (it == elements.end())
      elements.push_back({yaw, {{time, node}}});
    else
      it->time_map.insert({time, node});
  }

  std::vector<Element> elements;
};
} // anonymous namespace

//==============================================================================
template<typename NodePtr>
std::vector<NodePtr> reconstruct_nodes(
  const NodePtr& finish_node,
  const agv::RouteValidator* validator)
{
  auto node_sequence = reconstruct_nodes(finish_node);

  // Remove "cruft" from plans. This means making sure vehicles don't do any
  // unnecessary motions.
  std::unordered_map<
    std::size_t,
    OrientationTimeMap<NodePtr>
  > cruft_map;

  for (const auto& node : node_sequence)
  {
    if (!node->waypoint)
      continue;

    const auto wp = *node->waypoint;
    cruft_map[wp].insert(node);
  }

  for (auto& cruft : cruft_map)
  {
    for (auto& duplicate : cruft.second.elements)
      duplicate.squash(validator);
  }

  return reconstruct_nodes(finish_node);
}

//==============================================================================
struct Indexing
{
  std::size_t itinerary_index;
  std::size_t trajectory_index;
};

//==============================================================================
template<typename NodePtr>
std::pair<std::vector<Route>, std::vector<Indexing>> reconstruct_routes(
  const std::vector<NodePtr>& node_sequence,
  rmf_utils::optional<rmf_traffic::Duration> span = rmf_utils::nullopt)
{
  if (node_sequence.size() == 1)
  {
    std::vector<Route> output;
    // If there is only one node in the sequence, then it is a start node.
    if (span)
    {
      // When performing a rollout, it is important that at least one route with
      // two waypoints is provided. We use the span value to creating a
      // stationary trajectory when the robot is already starting out at a
      // holding point.

      // TODO(MXG): Make a unit test for this situation
      std::vector<Route> simple_route =
        node_sequence.back()->route_from_parent;
      if (simple_route.back().trajectory().size() < 2)
      {
        const auto& wp = simple_route.back().trajectory().back();
        for (auto&  r : simple_route)
        {
          r.trajectory().insert(
            wp.time() + *span,
            wp.position(),
            Eigen::Vector3d::Zero());
        }
      }

      output = std::move(simple_route);
    }

    // When there is only one node, we should return an empty itinerary to
    // indicate that the AGV does not need to go anywhere.
    return {output, {}};
  }

  std::vector<Indexing> indexing;
  indexing.resize(node_sequence.size());

  assert(node_sequence.back()->start.has_value());

  std::vector<Route> routes = node_sequence.back()->route_from_parent;
  indexing.at(node_sequence.size()-1).itinerary_index = 0;
  indexing.at(node_sequence.size()-1).trajectory_index = 0;

  // We exclude the first node in the sequence, because it contains an empty
  // route which is not helpful.
  const std::size_t N = node_sequence.size();
  const std::size_t stop = node_sequence.size()-1;
  for (std::size_t i = 0; i < stop; ++i)
  {
    const std::size_t n = N-2-i;
    const auto& node = node_sequence.at(n);
    auto& index = indexing.at(n);

    for (const Route& next_route : node->route_from_parent)
    {
      Route& last_route = routes.back();
      if (next_route.map() == last_route.map())
      {
        for (const auto& waypoint : next_route.trajectory())
        {
          last_route.trajectory().insert(waypoint);
        }
      }
      else
      {
        routes.push_back(next_route);
      }

      // We will take note of the itinerary and trajectory indices here
      index.itinerary_index = routes.size() - 1;
      assert(!routes.back().trajectory().empty());
      index.trajectory_index = routes.back().trajectory().size() - 1;
    }


    assert(routes.back().trajectory().back().time() == node->time);
  }

  // Throw away any routes that have less than two waypoints.
  const auto r_it = std::remove_if(
    routes.begin(), routes.end(),
    [](const auto& r) { return r.trajectory().size() < 2; });
  routes.erase(r_it, routes.end());

  return {routes, indexing};
}

//==============================================================================
template<typename NodePtr>
std::vector<Plan::Waypoint> reconstruct_waypoints(
  const std::vector<NodePtr>& node_sequence,
  const std::vector<Indexing>& indexing,
  const Graph::Implementation& graph)
{
  if (node_sequence.size() == 1)
  {
    // If there is only one node in the sequence, then it is a start node, and
    // it implies that no plan is actually needed.
    assert(node_sequence.front()->start.has_value());
    return {};
  }

  assert(node_sequence.size() == indexing.size());
  const std::size_t N = node_sequence.size();

  std::vector<agv::Plan::Waypoint> waypoints;
  for (std::size_t i = 0; i < N; ++i)
  {
    const auto& n = node_sequence[N-1-i];
    const auto& index = indexing[N-1-i];

    const Eigen::Vector2d p = n->waypoint ?
      graph.waypoints[*n->waypoint].get_location() :
      n->route_from_parent.back().trajectory().back().position()
      .template block<2, 1>(0, 0);
    const Time time{*n->route_from_parent.back().trajectory().finish_time()};
    waypoints.emplace_back(
      Plan::Waypoint::Implementation::make(
        Eigen::Vector3d{p[0], p[1], n->yaw}, time, n->waypoint,
        n->approach_lanes, index.itinerary_index, index.trajectory_index,
        n->event));
  }

  return waypoints;
}

//==============================================================================
template<typename NodePtr>
Plan::Start find_start(NodePtr node)
{
  while (node->parent)
    node = node->parent;

  if (!node->start.has_value())
  {
    throw std::runtime_error(
            "[rmf_traffic::agv::Planner::plan] The root node of a solved plan is "
            "missing its Start information. This should not happen. Please report "
            "this critical bug to the maintainers of rmf_traffic.");
  }

  return node->start.value();
}

//==============================================================================
class ScheduledDifferentialDriveExpander
{
public:

  using Entry = DifferentialDriveMapTypes::Entry;
  using EntryHash = DifferentialDriveMapTypes::EntryHash;

  struct SearchNode;
  using SearchNodePtr = std::shared_ptr<SearchNode>;
  using ConstSearchNodePtr = std::shared_ptr<const SearchNode>;
  using NodePtr = SearchNodePtr;

  struct SearchNode
  {
    // We use optional here because start nodes don't always have an Entry value
    // or a waypoint. If this is a nullopt, then SearchNode::start should have a
    // value.
    std::optional<Entry> entry;
    std::optional<std::size_t> waypoint;
    std::vector<std::size_t> approach_lanes;
    Eigen::Vector2d position;
    double yaw;
    Time time;

    double remaining_cost_estimate;

    std::vector<Route> route_from_parent;

    // An event that should occur when this node is reached,
    // i.e. after route_from_parent has been traversed
    Graph::Lane::EventPtr event;

    double current_cost;
    std::optional<Planner::Start> start;
    SearchNodePtr parent;

    double get_total_cost_estimate() const
    {
      return current_cost + remaining_cost_estimate;
    }

    double get_remaining_cost_estimate() const
    {
      return remaining_cost_estimate;
    }

    std::optional<Orientation> get_orientation() const
    {
      if (entry.has_value())
        return entry->orientation;

      return std::nullopt;
    }

    SearchNode(
      std::optional<Entry> entry_,
      std::optional<std::size_t> waypoint_,
      std::vector<std::size_t> approach_lanes_,
      Eigen::Vector2d position_,
      double yaw_,
      Time time_,
      double remaining_cost_estimate_,
      std::vector<Route> route_from_parent_,
      Graph::Lane::EventPtr event_,
      double current_cost_,
      std::optional<Planner::Start> start_,
      SearchNodePtr parent_)
    : entry(entry_),
      waypoint(waypoint_),
      approach_lanes(std::move(approach_lanes_)),
      position(position_),
      yaw(yaw_),
      time(time_),
      remaining_cost_estimate(remaining_cost_estimate_),
      route_from_parent(std::move(route_from_parent_)),
      event(event_),
      current_cost(current_cost_),
      start(std::move(start_)),
      parent(std::move(parent_))
    {
      assert(!route_from_parent.empty());

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      if (parent &&
        (parent->get_total_cost_estimate() > get_total_cost_estimate() + 1e-3))
      {
        std::cout << "Inadmissible expansion! "
                  << parent->current_cost << " + "
                  << parent->remaining_cost_estimate << " = "
                  << parent->get_total_cost_estimate()
                  << " --> "
                  << current_cost << " + "
                  << remaining_cost_estimate << " = "
                  << get_total_cost_estimate()
                  << " | Diff: "
                  << parent->get_total_cost_estimate() -
          get_total_cost_estimate()
                  << std::endl;

        std::cout << "Yaw: " << parent->yaw*180.0/M_PI << " --> "
                  << yaw*180.0/M_PI << " | Trans: ("
                  << parent->position.transpose() << ") --> ("
                  << position.transpose() << ") <"
                  << (parent->position - position).norm() << ">" << std::endl;
      }
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      if (start.has_value())
      {
        std::cout << "making start node (" << route_from_parent.size()
                  << "):";
        for (const auto& r : route_from_parent)
          std::cout << " " << r.trajectory().size();
        std::cout << std::endl;

        std::cout << " -- [";
        if (waypoint)
          std::cout << waypoint.value();
        else
          std::cout << "nullopt";

        std::cout << ":" << start->waypoint() << "] -- <"
                  << position.transpose() << "; " << yaw << "> vs <";

        if (start->location().has_value())
          std::cout << start->location()->transpose();
        else
          std::cout << "nullopt";

        std::cout << "; " << start->orientation() << ">" << std::endl;
      }
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
    }
  };

  using SearchQueue =
    std::priority_queue<
    SearchNodePtr,
    std::vector<SearchNodePtr>,
    DifferentialDriveCompare<SearchNodePtr>
    >;

  class InternalState : public State::Internal
  {
  public:

    std::optional<double> cost_estimate() const final
    {
      if (queue.empty())
        return std::nullopt;

      const auto& top = queue.top();
      return top->current_cost + top->remaining_cost_estimate;
    }

    std::size_t queue_size() const final
    {
      return queue.size();
    }

    std::size_t expansion_count() const final
    {
      return popped_count;
    }

    SearchQueue queue;
    std::size_t popped_count = 0;
  };

  bool quit(const SearchNodePtr& top, SearchQueue& queue) const
  {
    ++_internal->popped_count;

    if (_saturation_limit.has_value())
    {
      if (*_saturation_limit < _internal->popped_count + queue.size())
        return true;
    }

    if (_maximum_cost_estimate.has_value())
    {
      const double cost_estimate = top->get_total_cost_estimate();

      if (*_maximum_cost_estimate < cost_estimate)
        return true;
    }

    if (_interrupter && _interrupter())
    {
      _issues->interrupted = true;
      return true;
    }

    return false;
  }

  bool is_at_goal(const SearchNodePtr& top) const
  {
    if (top->waypoint == _goal_waypoint)
    {
      if (!_goal_yaw.has_value())
        return true;

      const double angle_diff = rmf_utils::wrap_to_pi(top->yaw - *_goal_yaw);
      if (std::abs(angle_diff) <= _rotation_threshold)
        return true;
    }

    return false;
  }

  bool is_finished(const SearchNodePtr& top) const
  {
    if (_goal_time.has_value())
    {
      if (top->time < *_goal_time)
        return false;
    }

    return is_at_goal(top);
  }

  void expand_start(const SearchNodePtr& top, SearchQueue& queue) const
  {
    const auto& start = top->start.value();
    const std::size_t target_waypoint_index = start.waypoint();
    const auto& wp = _supergraph->original().waypoints[target_waypoint_index];
    const Eigen::Vector2d wp_location = wp.get_location();

    // If this start node did not have a waypoint, then it must have a location
    assert(start.location().has_value());

    const auto approach_info = make_start_approach_trajectories(
      top->start.value(), top->current_cost);

    if (approach_info.trajectories.empty())
    {
      // This means there are no valid ways to approach the start. We should
      // just give up on expanding from this node.
      return;
    }

    std::vector<std::string> map_names;
    std::vector<std::size_t> approach_lanes;
    Graph::Lane::EventPtr exit_event;
    double exit_event_cost = 0.0;
    Duration exit_event_duration = Duration(0);
    if (const auto lane_index = start.lane())
    {
      approach_lanes.push_back(*lane_index);
      const auto& lane = _supergraph->original().lanes[*lane_index];

      const std::size_t wp0_index = lane.entry().waypoint_index();
      const auto& wp0 = _supergraph->original().waypoints[wp0_index];
      const auto& wp0_map = wp0.get_map_name();

      assert(lane.exit().waypoint_index() == target_waypoint_index);
      const auto& wp1_map = wp.get_map_name();

      map_names.push_back(wp0_map);
      if (wp0_map != wp1_map)
        map_names.push_back(wp1_map);

      if (lane.exit().event())
      {
        exit_event = lane.exit().event()->clone();
        exit_event_duration = exit_event->duration();
        exit_event_cost = time::to_seconds(exit_event_duration);
      }
    }
    else
    {
      map_names.push_back(wp.get_map_name());
    }

    for (const auto& approach_trajectory : approach_info.trajectories)
    {
      std::vector<Route> approach_routes;
      bool all_valid = true;

      for (const auto& map : map_names)
      {
        Route route{map, approach_trajectory};
        if (_validator && !is_valid(top, route))
        {
          all_valid = false;
          break;
        }

        approach_routes.emplace_back(std::move(route));
      }

      if (!all_valid)
        continue;

      std::vector<Route> exit_event_routes;
      if (exit_event)
      {
        Trajectory hold;
        const auto& approached = approach_trajectory.back();
        hold.insert(approached);
        hold.insert(approached.time(), approached.position(), {0, 0, 0});

        bool all_valid = true;
        for (const auto& map : map_names)
        {
          Route route{map, hold};
          if (_validator && !is_valid(top, route))
          {
            all_valid = false;
            break;
          }

          exit_event_routes.emplace_back(std::move(route));
        }

        if (!all_valid)
          continue;
      }

      const double approach_cost = time::to_seconds(
        approach_trajectory.duration());
      const double approach_yaw = approach_trajectory.back().position()[2];
      const auto approach_time = *approach_trajectory.finish_time();

      // TODO(MXG): We can actually specify the orientation for this. We just
      // need to be smarter with make_start_approach_trajectories(). We should
      // really have it return a Traversal.
      auto node = std::make_shared<SearchNode>(
        SearchNode{
          std::nullopt,
          target_waypoint_index,
          approach_lanes,
          wp_location,
          approach_yaw,
          approach_time,
          top->remaining_cost_estimate - approach_cost,
          std::move(approach_routes),
          exit_event,
          top->current_cost + approach_cost,
          std::nullopt,
          top
        });

      if (exit_event)
      {
        node = std::make_shared<SearchNode>(
          SearchNode{
            std::nullopt,
            target_waypoint_index,
            {},
            wp_location,
            approach_yaw,
            approach_time + exit_event_duration,
            node->remaining_cost_estimate - exit_event_cost,
            std::move(exit_event_routes),
            nullptr,
            node->current_cost + exit_event_cost,
            std::nullopt,
            node
          });
      }

      queue.push(node);
    }

    const Time initial_time = top->time;
    const Time hold_until = initial_time + _holding_time;
    const double hold_cost = time::to_seconds(_holding_time);
    const Eigen::Vector2d p = top->position;
    const double yaw = top->yaw;
    const Eigen::Vector3d position{p.x(), p.y(), yaw};
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    Trajectory hold;
    hold.insert(initial_time, position, zero);
    hold.insert(hold_until, position, zero);

    std::vector<Route> hold_routes;
    for (const auto& map : map_names)
    {
      Route route{map, hold};
      if (_validator && !is_valid(top, route))
        return;

      hold_routes.emplace_back(std::move(route));
    }

    queue.push(
      std::make_shared<SearchNode>(
        SearchNode{
          std::nullopt,
          std::nullopt,
          {},
          p,
          yaw,
          hold_until,
          top->remaining_cost_estimate,
          std::move(hold_routes),
          nullptr,
          top->current_cost + hold_cost,
          start,
          top
        }));
  }

  SearchNodePtr expand_hold(
    const SearchNodePtr& top,
    const Duration hold_time,
    const double cost_factor) const
  {
    const std::size_t wp_index = top->waypoint.value();
    if (_supergraph->original().waypoints[wp_index].is_passthrough_point())
      return nullptr;

    const std::string& map_name =
      _supergraph->original().waypoints[wp_index].get_map_name();

    const Eigen::Vector2d p = top->position;
    const double yaw = top->yaw;
    const Eigen::Vector3d position{p.x(), p.y(), yaw};
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    const auto start_time = top->time;
    const auto finish_time = start_time + hold_time;
    const auto cost = cost_factor * time::to_seconds(hold_time);

    Trajectory trajectory;
    trajectory.insert(start_time, position, zero);
    trajectory.insert(finish_time, position, zero);

    Route route{map_name, std::move(trajectory)};

    if (!is_valid(top, route))
      return nullptr;

    return std::make_shared<SearchNode>(
      SearchNode{
        top->entry,
        wp_index,
        {},
        p,
        yaw,
        finish_time,
        top->remaining_cost_estimate,
        {std::move(route)},
        nullptr,
        top->current_cost + cost,
        std::nullopt,
        top
      });
  }

  void expand_hold(
    const SearchNodePtr& top,
    SearchQueue& queue) const
  {
    if (const auto node = expand_hold(top, _holding_time, 1.0))
    {
      if (_should_expand_to(node))
        queue.push(node);
    }
  }

  SearchNodePtr rotate_to_goal(const SearchNodePtr& top) const
  {
    assert(top->waypoint == _goal_waypoint);
    const std::string& map_name =
      _supergraph->original().waypoints[_goal_waypoint].get_map_name();

    const Eigen::Vector2d p = top->position;
    const double target_yaw = _goal_yaw.value();
    const Eigen::Vector3d start_position{p.x(), p.y(), top->yaw};
    const auto start_time = top->time;

    const Eigen::Vector3d finish_position{p.x(), p.y(), target_yaw};

    Trajectory trajectory;
    trajectory.insert(
      start_time, start_position, Eigen::Vector3d::Zero());
    internal::interpolate_rotation(
      trajectory, _w_nom, _alpha_nom, start_time,
      start_position, finish_position, _rotation_threshold);

    assert(trajectory.size() >= 2);

    const auto finish_time = *trajectory.finish_time();
    const double cost = time::to_seconds(trajectory.duration());
    Route route{map_name, std::move(trajectory)};
    if (_validator && !is_valid(top, route))
      return nullptr;

    return std::make_shared<SearchNode>(
      SearchNode{
        std::nullopt,
        _goal_waypoint,
        {},
        p,
        target_yaw,
        finish_time,
        0.0,
        {std::move(route)},
        nullptr,
        top->current_cost + cost,
        std::nullopt,
        top
      });
  }

  bool is_valid(const SearchNodePtr& parent, const Route& route) const
  {
    if (!_validator)
      return true;

    if (route.trajectory().size() >= 2)
    {
      auto conflict = _validator->find_conflict(route);
      if (conflict)
      {
        auto time_it =
          _issues->blocked_nodes[conflict->participant]
          .insert({parent, conflict->time});

        if (!time_it.second)
        {
          time_it.first->second =
            std::max(time_it.first->second, conflict->time);
        }

        return false;
      }
    }

    return true;
  }

  void expand_traversal(
    const SearchNodePtr& top,
    const Traversal& traversal,
    SearchQueue& queue) const
  {
    const auto initial_waypoint_index = top->waypoint.value();
    const auto& initial_waypoint =
      _supergraph->original().waypoints[initial_waypoint_index];
    const Eigen::Vector2d p0 = initial_waypoint.get_location();
    const double initial_yaw = top->yaw;
    const std::string& initial_map_name = initial_waypoint.get_map_name();

    const auto next_waypoint_index = traversal.finish_waypoint_index;
    const auto& next_waypoint =
      _supergraph->original().waypoints[next_waypoint_index];
    const Eigen::Vector2d next_position = next_waypoint.get_location();
    const std::string& next_map_name = next_waypoint.get_map_name();

    for (std::size_t i = 0; i < traversal.alternatives.size(); ++i)
    {
      const auto& alt = traversal.alternatives[i];

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << "Expanding from " << top->waypoint.value()
                << " -> " << traversal.finish_waypoint_index << " | "
                << Orientation(i) << " {" << traversal.entry_event << "}"
                << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      if (!alt.has_value())
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== nullopt alternative" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      const Orientation orientation = Orientation(i);

      Time start_time = top->time;
      const auto traversal_yaw = alt->yaw;

      Trajectory approach_trajectory;
      const Eigen::Vector3d start{p0.x(), p0.y(), initial_yaw};
      approach_trajectory.insert(
        start_time, start, Eigen::Vector3d::Zero());

      // TODO(MXG): We could push the logic for creating this trajectory
      // upstream into the traversal alternative.
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      double approach_cost = 0.0;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      if (traversal_yaw.has_value())
      {
        const Eigen::Vector3d finish{p0.x(), p0.y(), * traversal_yaw};
        internal::interpolate_rotation(
          approach_trajectory, _w_nom, _alpha_nom, start_time,
          start, finish, _rotation_threshold);

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        approach_cost = time::to_seconds(approach_trajectory.duration());
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      }

      auto approach_route =
        Route{
        initial_map_name,
        std::move(approach_trajectory)
      };

      if (!is_valid(top, approach_route))
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== Invalid approach route" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      Trajectory entry_event_trajectory;
      const auto& approach_wp = approach_route.trajectory().back();
      entry_event_trajectory.insert(approach_wp);
      double entry_event_cost = 0.0;
      if (traversal.entry_event
        && traversal.entry_event->duration() > Duration(0))
      {
        const auto duration = traversal.entry_event->duration();
        entry_event_cost = time::to_seconds(duration);

        entry_event_trajectory.insert(
          approach_wp.time() + duration,
          approach_wp.position(), Eigen::Vector3d::Zero());
      }

      auto entry_event_route =
        Route{
        initial_map_name,
        std::move(entry_event_trajectory)
      };

      if (!is_valid(top, entry_event_route))
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== Invalid entry event route" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      const auto& ready_wp = entry_event_route.trajectory().back();
      const auto ready_time = ready_wp.time();
      const double ready_yaw = ready_wp.position()[2];
      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);

      bool all_valid = true;
      for (const auto& r : traversal_result.routes)
      {
        if (!is_valid(top, r))
        {
          all_valid = false;
          break;
        }
      }

      if (!all_valid)
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== Invalid traversal" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << " --------" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      const auto remaining_cost_estimate = _heuristic.compute(
        next_waypoint_index, traversal_result.finish_yaw);

      if (!remaining_cost_estimate.has_value())
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== nullopt heuristic" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      const auto& arrival_wp =
        traversal_result.routes.back().trajectory().back();

      Trajectory exit_event_trajectory;
      exit_event_trajectory.insert(arrival_wp);
      double exit_event_cost = 0.0;
      Duration exit_event_duration = Duration(0);
      if (traversal.exit_event
        && traversal.exit_event->duration() > Duration(0))
      {
        exit_event_duration = traversal.exit_event->duration();
        exit_event_cost = time::to_seconds(exit_event_duration);

        exit_event_trajectory.insert(
          arrival_wp.time() + exit_event_duration,
          arrival_wp.position(), Eigen::Vector3d::Zero());
      }

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << "Cost " << approach_cost + entry_event_cost + alt->time
        + exit_event_cost << " = " << "Approach: " << approach_cost
                << " | Entry: " << entry_event_cost << " | Alt: " << alt->time
                << " | Exit: " << exit_event_cost << std::endl;
      std::cout << "Previous cost " << top->current_cost << " + Cost "
                << approach_cost + entry_event_cost + alt->time
        + exit_event_cost << " = " << top->current_cost
        + approach_cost + entry_event_cost + alt->time
        + exit_event_cost << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      auto exit_event_route =
        Route{
        next_map_name,
        std::move(exit_event_trajectory)
      };

      if (!is_valid(top, exit_event_route))
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== invalid exit event" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      auto node = top;
      if (approach_route.trajectory().size() >= 2 || traversal.entry_event)
      {
        const double cost =
          time::to_seconds(approach_route.trajectory().duration());
        const double yaw = approach_wp.position()[2];
        const auto time = approach_wp.time();

        node = std::make_shared<SearchNode>(
          SearchNode{
            Entry{
              traversal.initial_lane_index,
              orientation,
              Side::Start
            },
            initial_waypoint_index,
            {},
            p0,
            yaw,
            time,
            *remaining_cost_estimate
            + entry_event_cost + alt->time + exit_event_cost,
            {std::move(approach_route)},
            traversal.entry_event,
            node->current_cost + cost,
            std::nullopt,
            node
          });
      }

      if (entry_event_route.trajectory().size() >= 2)
      {
        auto& front = traversal_result.routes.front();
        if (entry_event_route.map() == front.map())
        {
          for (const auto& wp : entry_event_route.trajectory())
            front.trajectory().insert(wp);
        }
        else
        {
          traversal_result.routes.insert(
            traversal_result.routes.begin(),
            entry_event_route);
        }
      }

      const Entry finish_key = Entry{
        traversal.initial_lane_index,
        orientation,
        Side::Finish
      };

      node = std::make_shared<SearchNode>(
        SearchNode{
          traversal.exit_event ? std::nullopt : std::make_optional(finish_key),
          next_waypoint_index,
          traversal.traversed_lanes,
          next_position,
          traversal_result.finish_yaw,
          traversal_result.finish_time,
          *remaining_cost_estimate + exit_event_cost,
          std::move(traversal_result.routes),
          traversal.exit_event,
          node->current_cost + entry_event_cost + alt->time,
          std::nullopt,
          node
        });

      if (traversal.exit_event && exit_event_route.trajectory().size() >= 2)
      {
        node = std::make_shared<SearchNode>(
          SearchNode{
            finish_key,
            next_waypoint_index,
            {},
            next_position,
            traversal_result.finish_yaw,
            traversal_result.finish_time + exit_event_duration,
            *remaining_cost_estimate,
            {std::move(exit_event_route)},
            nullptr,
            node->current_cost + exit_event_cost,
            std::nullopt,
            node
          });
      }

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << " ^^^^^^^^^^^^^^ Pushing" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      if (_should_expand_to(node))
        queue.push(node);
    }
  }

  void expand_freely(
    const SearchNodePtr& top,
    SearchQueue& queue) const
  {
    // This function is used when there is no validator. We can just expand
    // freely to the goal without validating the results.
    const auto keys = _supergraph->keys_for(
      top->waypoint.value(), _goal_waypoint, _goal_yaw);

    for (const auto& key : keys)
    {
      const auto solution_root = _heuristic.cache().get(key);
      if (!solution_root)
        continue;

      auto search_node = top;

      auto approach_info = solution_root->route_factory(top->time, top->yaw);
      if (approach_info.routes.back().trajectory().size() >= 2
        || solution_root->info.event)
      {
        // TODO(MXG): Refactor this logic into a conversion function
        const auto cost =
          time::to_seconds(approach_info.finish_time - top->time);

        search_node = std::make_shared<SearchNode>(
          SearchNode{
            solution_root->info.entry,
            solution_root->info.waypoint,
            solution_root->info.approach_lanes,
            solution_root->info.position,
            approach_info.finish_yaw,
            approach_info.finish_time,
            solution_root->info.remaining_cost_estimate,
            std::move(approach_info.routes),
            solution_root->info.event,
            search_node->current_cost + cost,
            std::nullopt,
            search_node
          });
      }

      auto solution_node = solution_root->child;

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
//      std::cout << "Free solution: ";
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      while (solution_node)
      {
        assert(solution_node->route_factory);

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << "(" << search_node->current_cost << "; ";
        if (search_node->waypoint.has_value())
          std::cout << top->waypoint.value();
        else
          std::cout << "null";

        std::cout << ", " << search_node->yaw << ") ";
        if (solution_node->info.entry.has_value())
          std::cout << *solution_node->info.entry;
        else
          std::cout << "[null]";

        std::cout << " <" << solution_node->info.cost_from_parent
                  << " : " << solution_node->info.remaining_cost_estimate
                  << "> --> ";
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        auto route_info = solution_node->route_factory(
          search_node->time, search_node->yaw);

        search_node = std::make_shared<SearchNode>(
          SearchNode{
            solution_node->info.entry,
            solution_node->info.waypoint,
            solution_node->info.approach_lanes,
            solution_node->info.position,
            route_info.finish_yaw,
            route_info.finish_time,
            solution_node->info.remaining_cost_estimate,
            std::move(route_info.routes),
            solution_node->info.event,
            search_node->current_cost + solution_node->info.cost_from_parent,
            std::nullopt,
            search_node
          });

        solution_node = solution_node->child;
      }

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      queue.push(search_node);
    }
  }

  void expand(const SearchNodePtr& top, SearchQueue& queue) const
  {
    if (!_should_expand_from(top))
    {
      // This means we have already expanded from this location before, at
      // approximately the same time, so there is no value in expanding this
      // again.
      return;
    }

    if (!top->waypoint.has_value())
    {
      // If the node does not have a waypoint, then it must be a start node.
      assert(top->start.has_value());
      expand_start(top, queue);
      return;
    }

    const auto current_wp_index = top->waypoint.value();
    if (is_at_goal(top))
    {
      // If there is no goal time, then is_finished should have caught this node
      assert(_goal_time.has_value());

      const Duration remaining_time = _goal_time.value() - top->time;

      const auto finishing_node = expand_hold(top, remaining_time, 0.0);
      if (finishing_node)
      {
        // If we can reach the finish time by just sitting here, then that will
        // surely be the optimal solution. We will simply push this new node
        // into the queue and return.
        queue.push(finishing_node);
        return;
      }

      // If we cannot hold all the way until the finishing time, then we will
      // do a zero-cost brief hold so that we spend as much time waiting on the
      // goal as allowed.
      if (const auto brief_hold = expand_hold(top, _holding_time, 0.0))
        queue.push(brief_hold);
    }
    else if (current_wp_index == _goal_waypoint)
    {
      // If there is no goal yaw, then is_at_goal should have caught this node
      assert(_goal_yaw.has_value());

      if (auto node = rotate_to_goal(top))
        queue.push(std::move(node));

      if (_validator)
        expand_hold(top, queue);
    }
    else if (_validator)
    {
      // There will never be a reason to hold if there is no validator.
      expand_hold(top, queue);
    }

    if (!_validator)
    {
      // If we don't have a validator, then we can jump straight to the solution
      expand_freely(top, queue);
      return;
    }

    const auto traversals = _supergraph->traversals_from(current_wp_index);
    for (const auto& traversal : *traversals)
      expand_traversal(top, traversal, queue);
  }

  struct ApproachInfo
  {
    bool need_approach;
    std::vector<Trajectory> trajectories;
  };

  ApproachInfo make_start_approach_trajectories(
    const Planner::Start& start,
    const double hold_time) const
  {
    // TODO(MXG): We could stash ApproachInfo into the start node to avoid
    // needing to regenerate these trajectories repeatedly.

    const auto location_opt = start.location();
    if (!location_opt.has_value())
      return {false, {}};

    // This will return all the different trajectories that can be used to
    // approach the start. If it returns empty, that means either you forgot to
    // check the location field, or there are no valid ways to approach the
    // start.

    const auto* differential = _supergraph->traits().get_differential();
    DifferentialDriveConstraint constraint{
      differential->get_forward(),
      differential->is_reversible()
    };

    const std::size_t waypoint_index = start.waypoint();

    const Eigen::Vector2d p0 = *location_opt;
    const Eigen::Vector2d p1 =
      _supergraph->original().waypoints[waypoint_index].get_location();

    const double translation_thresh = _supergraph->options().translation_thresh;

    const double dist = (p1 - p0).norm();
    if (dist < translation_thresh)
    {
      // No trajectory is really needed.
      return {false, {}};
    }

    const Eigen::Vector2d course_vector = (p1 - p0)/dist;
    const auto yaw_options = constraint.get_orientations(course_vector);

    const Graph::Lane* const lane = start.lane() ?
      &_supergraph->original().lanes[*start.lane()] : nullptr;
    const Graph::OrientationConstraint* const entry_constraint =
      lane ? lane->entry().orientation_constraint() : nullptr;
    const Graph::OrientationConstraint* const exit_constraint =
      lane ? lane->exit().orientation_constraint() : nullptr;

    const double rotation_thresh = _supergraph->options().rotation_thresh;
    const auto start_time = start.time() + time::from_seconds(hold_time);
    const double start_yaw = start.orientation();
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

    const auto& traits = _supergraph->traits();
    const auto& linear = traits.linear();
    const auto& angular = traits.rotational();
    const double v_nom = linear.get_nominal_velocity();
    const double a_nom = linear.get_nominal_acceleration();
    const double w_nom = angular.get_nominal_velocity();
    const double alpha_nom = angular.get_nominal_acceleration();

    std::vector<Trajectory> trajectories;
    for (const auto& yaw_opt : yaw_options)
    {
      if (!yaw_opt.has_value())
        continue;

      const double yaw = *yaw_opt;

      if (!orientation_constraint_satisfied(
          p0, yaw, course_vector, entry_constraint, rotation_thresh))
        continue;

      if (!orientation_constraint_satisfied(
          p1, yaw, course_vector, exit_constraint, rotation_thresh))
        continue;

      Trajectory trajectory;
      const Eigen::Vector3d p_start = {p0.x(), p0.y(), start_yaw};
      trajectory.insert(start_time, p_start, zero);

      const Eigen::Vector3d p_oriented{p0.x(), p0.y(), yaw};
      internal::interpolate_rotation(
        trajectory, w_nom, alpha_nom, start_time, p_start, p_oriented,
        rotation_thresh);

      const Eigen::Vector3d p_arrived{p1.x(), p1.y(), yaw};
      internal::interpolate_translation(
        trajectory, v_nom, a_nom, *trajectory.finish_time(),
        p_oriented, p_arrived, translation_thresh);

      trajectories.emplace_back(std::move(trajectory));
    }

    return {true, std::move(trajectories)};
  }

  SearchNodePtr make_start_node(const Planner::Start& start) const
  {
    const std::size_t initial_waypoint_index = start.waypoint();
    const auto& initial_waypoint =
      _supergraph->original().waypoints.at(initial_waypoint_index);
    const auto& initial_map = initial_waypoint.get_map_name();

    const auto initial_time = start.time();
    const auto initial_yaw = start.orientation();
    double remaining_cost_estimate = 0.0;
    std::optional<std::size_t> node_waypoint;

    const Eigen::Vector2d waypoint_location =
      _supergraph->original().waypoints[initial_waypoint_index].get_location();

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
    std::cout << " >> Making start node " << start.waypoint() << ", "
              << start.orientation() << std::endl;
    if (start.location().has_value())
    {
      std::cout << "   > location: " << start.location().value().transpose()
                << std::endl;
    }
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

    Trajectory start_point_trajectory;
    const auto start_location = start.location();

    auto approach_info = make_start_approach_trajectories(start, 0.0);
    if (approach_info.need_approach)
    {
      std::optional<double> lowest_cost_estimate;
      for (const auto& approach : approach_info.trajectories)
      {
        const double yaw = approach.back().position()[2];
        double cost = time::to_seconds(approach.duration());
        const auto heuristic_cost_estimate =
          _heuristic.compute(initial_waypoint_index, yaw);

        if (!heuristic_cost_estimate.has_value())
          continue;

        cost += *heuristic_cost_estimate;
        if (!lowest_cost_estimate.has_value() || cost < *lowest_cost_estimate)
          lowest_cost_estimate = cost;
      }

      if (!lowest_cost_estimate.has_value())
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " === nullopt heuristic " << __LINE__ << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        // If this happens, the heuristic found that there is simply no path
        // from this start to the goal, so we return a nullptr.
        return nullptr;
      }

      remaining_cost_estimate = *lowest_cost_estimate;

      if (const auto lane_index = start.lane())
      {
        const auto& lane = _supergraph->original().lanes[*lane_index];
        if (const auto* exit_event = lane.exit().event())
          remaining_cost_estimate += time::to_seconds(exit_event->duration());
      }

      const Eigen::Vector3d start_position{
        start_location->x(),
        start_location->y(),
        initial_yaw
      };
      start_point_trajectory.insert(initial_time, start_position, {0, 0, 0});
    }
    else
    {
      node_waypoint = initial_waypoint_index;
      const auto heuristic_cost_estimate =
        _heuristic.compute(initial_waypoint_index, initial_yaw);

      if (!heuristic_cost_estimate.has_value())
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " === nullopt heuristic " << __LINE__ << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        // If this happens, the heuristic found that there is simply no path
        // from this start to the goal, so we return a nullptr.
        return nullptr;
      }

      remaining_cost_estimate = *heuristic_cost_estimate;

      const Eigen::Vector3d start_position{
        waypoint_location.x(),
        waypoint_location.y(),
        initial_yaw
      };
      start_point_trajectory.insert(initial_time, start_position, {0, 0, 0});
    }

    assert(!start_point_trajectory.empty());

    return std::make_shared<SearchNode>(
      SearchNode{
        std::nullopt,
        node_waypoint,
        {},
        start_location.value_or(waypoint_location),
        initial_yaw,
        start.time(),
        remaining_cost_estimate,
        {{initial_map, std::move(start_point_trajectory)}},
        nullptr,
        0.0,
        start,
        nullptr
      });
  }

  struct RolloutEntry
  {
    Time initial_time;
    SearchNodePtr node;

    bool operator==(const RolloutEntry& r) const
    {
      return node == r.node;
    }

    Duration span() const
    {
      return *node->route_from_parent.back().trajectory().finish_time()
        - initial_time;
    }
  };

  bool is_holding_point(const std::optional<std::size_t> waypoint_index) const
  {
    if (!waypoint_index.has_value())
      return false;

    return _supergraph->original().waypoints
      .at(*waypoint_index).is_holding_point();
  }

  std::vector<schedule::Itinerary> rollout(
    const Duration max_span,
    const Issues::BlockedNodes& nodes,
    std::optional<std::size_t> max_rollouts) const
  {
    std::vector<RolloutEntry> rollout_queue;
    for (const auto& void_node : nodes)
    {
      bool skip = false;
      const auto original_node =
        std::static_pointer_cast<SearchNode>(void_node.first);

      const auto original_t = void_node.second;

      // TODO(MXG): This filtering approach is not reliable as it could be.
      // Certain blockages will only be expanded half as far past the blockage
      // as the API implies. It would be better if each type of node expansion
      // could have a unique identifier so we could both avoid redundant
      // expansions while still expanding a blockage out as far as the API
      // says that it will.
//      const auto merge_span = max_span/2.0;

      auto ancestor = original_node->parent;
      while (ancestor)
      {
        if (nodes.count(ancestor) > 0)
        {
          // TODO(MXG): Consider if we should account for the time difference
          // between these conflicts so that we get a broader rollout.
//          const auto t = *ancestor->route_from_parent.trajectory.finish_time();
//          if (t - original_t < merge_span)
          {
            skip = true;
          }

          break;
        }

        ancestor = ancestor->parent;
      }

      if (skip)
        continue;

      rollout_queue.emplace_back(
        RolloutEntry{
          original_t,
          original_node
        });

      // TODO(MXG): Consider making this configurable, or making a more
      // meaningful decision on how to prune the initial rollout queue.
      if (rollout_queue.size() > 5)
        break;
    }

    std::unordered_map<NodePtr, ConstRoutePtr> route_map;
    std::vector<schedule::Itinerary> alternatives;

    Issues::BlockerMap temp_blocked_nodes;

    SearchQueue search_queue;
    SearchQueue finished_rollouts;

    while (!rollout_queue.empty() && !(_interrupter && _interrupter()))
    {
      const auto top = rollout_queue.back();
      rollout_queue.pop_back();

      const auto current_span = top.span();

      const bool stop_expanding =
        (max_span < current_span)
        || is_finished(top.node)
        || is_holding_point(top.node->waypoint);

      if (stop_expanding)
      {
        finished_rollouts.push(top.node);

        if (max_rollouts && *max_rollouts <= finished_rollouts.size())
          break;

        continue;
      }

      expand(top.node, search_queue);
      while (!search_queue.empty())
      {
        rollout_queue.emplace_back(
          RolloutEntry{
            top.initial_time,
            search_queue.top()
          });

        search_queue.pop();
      }
    }

    while (!finished_rollouts.empty())
    {
      auto node = finished_rollouts.top();
      finished_rollouts.pop();

      schedule::Itinerary itinerary;
      auto [routes, _] = reconstruct_routes(reconstruct_nodes(node), max_span);
      for (auto& r : routes)
      {
        assert(r.trajectory().size() > 0);
        itinerary.emplace_back(std::make_shared<Route>(std::move(r)));
      }

      assert(!itinerary.empty());
      alternatives.emplace_back(std::move(itinerary));
    }

    return alternatives;
  }

  ScheduledDifferentialDriveExpander(
    State::Internal* internal,
    Issues& issues,
    std::shared_ptr<const Supergraph> supergraph,
    DifferentialDriveHeuristicAdapter heuristic,
    const Planner::Goal& goal,
    const Planner::Options& options)
  : _internal(static_cast<InternalState*>(internal)),
    _issues(&issues),
    _supergraph(std::move(supergraph)),
    _heuristic(std::move(heuristic)),
    _goal_waypoint(goal.waypoint()),
    _goal_yaw(rmf_utils::pointer_to_opt(goal.orientation())),
    _goal_time(goal.minimum_time()),
    _validator(options.validator().get()),
    _holding_time(options.minimum_holding_time()),
    _discrete_time_window(_holding_time/2),
    _saturation_limit(options.saturation_limit()),
    _maximum_cost_estimate(options.maximum_cost_estimate()),
    _interrupter(options.interrupter()),
    _already_expanded(4093, EntryHash(_supergraph->original().lanes.size()))
  {
    const auto& angular = _supergraph->traits().rotational();
    _w_nom = angular.get_nominal_velocity();
    _alpha_nom = angular.get_nominal_acceleration();
    _rotation_threshold = _supergraph->options().rotation_thresh;
  }

  class Debugger : public Interface::Debugger
  {
  public:
    const Planner::Debug::Node::SearchQueue& queue() const final
    {
      return queue_;
    }

    const std::vector<Planner::Debug::ConstNodePtr>&
    expanded_nodes() const final
    {
      return expanded_nodes_;
    }

    const std::vector<Planner::Debug::ConstNodePtr>&
    terminal_nodes() const final
    {
      return terminal_nodes_;
    }

    NodePtr convert(agv::Planner::Debug::ConstNodePtr from)
    {
      auto output = _from_debug[from];
      assert(output);

      return output;
    }

    Planner::Debug::ConstNodePtr convert(NodePtr from)
    {
      const auto it = _to_debug.find(from);
      if (it != _to_debug.end())
        return it->second;

      std::vector<NodePtr> queue;
      queue.push_back(from);
      while (!queue.empty())
      {
        const auto node = queue.back();

        const auto parent = node->parent;
        agv::Planner::Debug::ConstNodePtr debug_parent = nullptr;
        if (parent)
        {
          const auto parent_it = _to_debug.find(parent);
          if (parent_it == _to_debug.end())
          {
            queue.push_back(parent);
            continue;
          }

          debug_parent = parent_it->second;
        }

        auto new_debug_node = std::make_shared<Planner::Debug::Node>(
          agv::Planner::Debug::Node{
            debug_parent,
            node->route_from_parent,
            node->remaining_cost_estimate,
            node->current_cost,
            node->waypoint,
            node->yaw,
            node->event,
            std::nullopt,
            next_id++
          });

        _to_debug[node] = new_debug_node;
        _from_debug[new_debug_node] = node;
        queue.pop_back();
      }

      return _to_debug[from];
    }

    agv::Planner::Debug::Node::SearchQueue queue_;
    std::vector<agv::Planner::Debug::ConstNodePtr> expanded_nodes_;
    std::vector<agv::Planner::Debug::ConstNodePtr> terminal_nodes_;
    Issues::BlockerMap blocked_nodes_;

    std::vector<agv::Planner::Start> starts_;
    agv::Planner::Goal goal_;
    agv::Planner::Options options_;

    std::size_t next_id = 0;

    Debugger(
      std::vector<agv::Planner::Start> starts,
      agv::Planner::Goal goal,
      agv::Planner::Options options)
    : starts_(std::move(starts)),
      goal_(std::move(goal)),
      options_(std::move(options))
    {
      // Do nothing
    }

    std::optional<PlanData> step(
      std::shared_ptr<const Supergraph> supergraph,
      Cache<DifferentialDriveHeuristic> cache)
    {
      InternalState internal;
      Issues issues;

      ScheduledDifferentialDriveExpander expander{
        &internal,
        issues,
        supergraph,
        DifferentialDriveHeuristicAdapter{
          cache,
          supergraph,
          goal_.waypoint(),
          rmf_utils::pointer_to_opt(goal_.orientation())
        },
        goal_,
        options_
      };

      return expander.debug_step(*this);
    }

  private:
    std::unordered_map<Planner::Debug::ConstNodePtr, NodePtr> _from_debug;
    std::unordered_map<NodePtr, Planner::Debug::ConstNodePtr> _to_debug;
  };

  std::unique_ptr<Interface::Debugger> debug_begin(
    const std::vector<agv::Planner::Start>& starts,
    agv::Planner::Goal goal,
    agv::Planner::Options options) const
  {
    auto debugger = std::make_unique<Debugger>(
      starts,
      std::move(goal),
      std::move(options));

    for (const auto& start : starts)
    {
      if (auto start_node = make_start_node(start))
        debugger->queue_.push(debugger->convert(std::move(start_node)));
    }

    return debugger;
  }

  std::optional<PlanData> debug_step(
    Interface::Debugger& input_debugger) const
  {
    Debugger& debugger = static_cast<Debugger&>(input_debugger);

    if (debugger.queue_.empty())
      return std::nullopt;

    auto top = debugger.convert(debugger.queue_.top());
    debugger.queue_.pop();
    debugger.expanded_nodes_.push_back(debugger.convert(top));

    if (is_finished(top))
      return make_plan(top);

    SearchQueue queue;
    expand(top, queue);

    if (queue.empty())
    {
      debugger.terminal_nodes_.push_back(debugger.convert(top));
    }
    else
    {
      while (!queue.empty())
      {
        debugger.queue_.push(debugger.convert(queue.top()));
        queue.pop();
      }
    }

    return rmf_utils::nullopt;
  }

  PlanData make_plan(const SearchNodePtr& solution) const
  {
    auto nodes = reconstruct_nodes(solution, _validator);
    auto [routes, index] = reconstruct_routes(nodes);
    auto waypoints = reconstruct_waypoints(
      nodes, index, _supergraph->original());
    auto start = find_start(solution);

    return PlanData{
      std::move(routes),
      std::move(waypoints),
      std::move(start),
      solution->current_cost
    };
  }

private:
  InternalState* _internal;
  Issues* _issues;
  std::shared_ptr<const Supergraph> _supergraph;
  DifferentialDriveHeuristicAdapter _heuristic;
  std::size_t _goal_waypoint;
  std::optional<double> _goal_yaw;
  std::optional<rmf_traffic::Time> _goal_time;
  const RouteValidator* _validator;
  Duration _holding_time;
  Duration _discrete_time_window;
  std::optional<std::size_t> _saturation_limit;
  std::optional<double> _maximum_cost_estimate;
  std::function<bool()> _interrupter;
  double _w_nom;
  double _alpha_nom;
  double _rotation_threshold;

  using TimeSet = std::set<rmf_traffic::Time>;
  using VisitMap = std::unordered_map<
    DifferentialDriveMapTypes::Entry,
    TimeSet,
    DifferentialDriveMapTypes::EntryHash
  >;

  mutable VisitMap _already_expanded;

  std::optional<TimeSet::const_iterator> _get_hint_if_not_redundant(
    const Time time,
    const TimeSet& time_set) const
  {
    const auto greater_or_equal_it = time_set.lower_bound(time);
    if (greater_or_equal_it == time_set.begin())
    {
      if (time + _discrete_time_window < *greater_or_equal_it)
      {
        return greater_or_equal_it;
      }

      // The node's time is within the tolerance of one that was already
      // expanded
      return std::nullopt;
    }

    const auto less_than = --TimeSet::const_iterator(greater_or_equal_it);
    if (time < *less_than + _discrete_time_window)
    {
      // The node is within the tolerance of an earlier time
      return std::nullopt;
    }

    // If we reach this point, the node is far enough from the earlier time, so
    // now we need to check if it's far enough from any later time.

    if (greater_or_equal_it == time_set.end())
    {
      // There is no need to check the node against a later time, because there
      // is no later time in the set.
      return greater_or_equal_it;
    }

    if (time + _discrete_time_window < *greater_or_equal_it)
    {
      return greater_or_equal_it;
    }

    return std::nullopt;
  }

  bool _should_expand_from(const SearchNodePtr& node) const
  {
    if (!node->entry.has_value())
      return true;

    const auto entry_it = _already_expanded.insert({*node->entry, {}}).first;
    TimeSet& time_set = entry_it->second;
    if (time_set.empty())
    {
      time_set.insert(node->time);
      return true;
    }

    const auto time = node->time;
    const auto hint = _get_hint_if_not_redundant(time, time_set);
    if (hint.has_value())
    {
      time_set.insert(*hint, time);
      return true;
    }

    return false;
  }

  bool _should_expand_to(const SearchNodePtr& node) const
  {
    if (!node->entry.has_value())
      return true;

    const auto entry_it = _already_expanded.insert({*node->entry, {}}).first;
    const TimeSet& time_set = entry_it->second;
    if (time_set.empty())
      return true;

    return _get_hint_if_not_redundant(node->time, time_set).has_value();
  }
};

//==============================================================================

CloberPlanner::CloberPlanner(Planner::Configuration config)
: _config(std::move(config))
{
  _supergraph = Supergraph::make(
    Graph::Implementation::get(_config.graph()), _config.vehicle_traits(),
    _config.lane_closures(), _config.interpolation());

  _cache = DifferentialDriveHeuristic::make_manager(_supergraph);

  std::cout <<"CloberPlanner Instance "<<std::endl;

  GenerateGraph();

  scheduler_ = std::make_shared<COrtools>();
  // std::string filename = "/home/clober/variables.yaml";
  // std::string filename = "/home/nuc/adapt/map_3x3/solver/variables.yaml";
  // scheduler_->LoadDataFrmFile(filename);
  GenerateMIPValues();
  scheduler_->PrintMIPData();

  testMIP();
}

void CloberPlanner::testMIP()
{
  RobotInfo target;
  target.robotId_ = "r0";
  target.start_ = "n5";
  target.end_ = "n9";
  target.path_.push_back("n5");
  target.path_.push_back("n6");
  target.path_.push_back("n9");

  RobotInfo enemy;
  enemy.robotId_ = "r1";
  enemy.start_ = "n6";
  enemy.end_ = "n6";
  enemy.path_.push_back("n6");
  enemy.startIdx_ = 0;

  std::vector<std::string> name_path = FindNewPath(target, enemy);
}

void CloberPlanner::GenerateMIPValues()
{
  std::vector<std::string> mip_node_set;
  std::vector<std::string> mip_arc_set;
  std::map<std::string, std::map<std::string, std::vector<std::string>>> mip_connect_set;
  
  std::cout <<" ~~ generate mip variables ~~ "<<std::endl;

  for (auto it = _config.graph().keys().begin();
    it != _config.graph().keys().end(); it ++)
  {
    mip_node_set.push_back(it->first);

    std::map<std::string, std::vector<std::string>> inner_set;
    std::vector<std::string> inner_set_in;
    std::vector<std::string> inner_set_out;
    std::vector<std::string> inner_set_stay;

    for (int i = 0; i < _config.graph().lanes_from(it->second).size(); i++)
    {
      std::size_t lane_idx = _config.graph().lanes_from(it->second)[i];
      Graph::Lane lane = _config.graph().get_lane(lane_idx);
      
      std::string entry_name = "";
      std::string exit_name = "";

      auto f_entry = id_nameGraph_.find(lane.entry().waypoint_index());
      if ( f_entry != id_nameGraph_.end() ){
        entry_name = f_entry->second;
      }else{
        // todo. std::cerr 로 변경
        std::cout << "error.. CloberPlanner::GenerateMIPValues()" << std::endl;
      }

      auto f_exit = id_nameGraph_.find(lane.exit().waypoint_index());
      if ( f_exit != id_nameGraph_.end() ){
        exit_name = f_exit->second;
      }else{
        // todo. std::cerr 로 변경
        std::cout << "error.. CloberPlanner::GenerateMIPValues()" << std::endl;
      }

      std::string arc_name;
      arc_name.push_back('a');
      arc_name.insert(arc_name.end(), entry_name.begin()+1, entry_name.end());
      arc_name.push_back('-');
      arc_name.insert(arc_name.end(), exit_name.begin()+1, exit_name.end());
      
      // add arc set ( entry-exit)
      mip_arc_set.push_back(arc_name);

      std::string in_name;
      in_name.push_back('a');
      in_name.insert(in_name.end(), exit_name.begin()+1, exit_name.end());
      in_name.push_back('-');
      in_name.insert(in_name.end(), entry_name.begin()+1, entry_name.end());

      std::string out_name;
      out_name.push_back('a');
      out_name.insert(out_name.end(), entry_name.begin()+1, entry_name.end());
      out_name.push_back('-');
      out_name.insert(out_name.end(), exit_name.begin()+1, exit_name.end());

      inner_set_in.push_back(in_name);
      inner_set_out.push_back(out_name);
    }

      std::string stay_name ="";
      std::string curr_name = it->first;
      stay_name.push_back('a');
      stay_name.insert(stay_name.end(), curr_name.begin()+1, curr_name.end());
      stay_name.push_back('-');
      stay_name.insert(stay_name.end(), curr_name.begin()+1, curr_name.end());
      
      // add arc set ( stay )
      mip_arc_set.push_back(stay_name);

      inner_set_stay.push_back(stay_name);

      inner_set.insert(std::make_pair("in", inner_set_in));
      inner_set.insert(std::make_pair("out", inner_set_out));
      inner_set.insert(std::make_pair("stay", inner_set_stay));

      mip_connect_set.insert(std::make_pair(it->first, inner_set));
  }

  scheduler_->LoadData(mip_node_set, mip_arc_set, mip_connect_set);
}



void CloberPlanner::GenerateGraph()
{

  std::cout <<"graph num waypoints : " << _config.graph().num_waypoints() <<
          std::endl;
  std::cout <<"graph num lane : " << _config.graph().num_lanes() <<std::endl;


  for (auto it = _config.graph().keys().begin();
    it != _config.graph().keys().end(); it ++)
  {
    std::cout << it->first << " , " << it->second <<std::endl;

    name_idGraph_.insert(std::make_pair(it->first, it->second));
    id_nameGraph_.insert(std::make_pair(it->second, it->first));

    for (int i = 0; i < _config.graph().lanes_from(it->second).size(); i++)
    {
      std::cout << it->second << " lane from : " << _config.graph().lanes_from(
          it->second)[i] << std::endl;

      // graph from wp_idx
      std::string fromIdx = std::to_string(it->second);
      auto fromIt = bfsGraph_.find(fromIdx);

      std::size_t lane_idx = _config.graph().lanes_from(it->second)[i];
      Graph::Lane lane = _config.graph().get_lane(lane_idx);
      std::cout << "lane idx " << lane_idx <<" , entry :  " <<
              lane.entry().waypoint_index() <<", exit : "<<
              lane.exit().waypoint_index() << std::endl;

      // graph to wp_idx
      std::string toIdx = std::to_string(lane.exit().waypoint_index());

      if (fromIt == bfsGraph_.end())
      {
        std::vector<std::string> nodelist;
        nodelist.push_back(toIdx);
        bfsGraph_.insert(make_pair(fromIdx, nodelist));
      }
      else
      {
        fromIt->second.push_back(toIdx);
      }
    }
      std::cout <<"~~~~~~~~~~~~~~~" << std::endl;
  }

  pbfs_ = std::make_shared<CBFS>(bfsGraph_);
  pbfs_->printGraph();

  std::string start("0");
  std::string end("3");
  testBFS(start, end);
}


void CloberPlanner::testBFS(std::string s, std::string g)
{
  std::vector<std::vector<std::string>> path;
  pbfs_->bfs_paths(s, g, path);
  path_ = path[0];
  pbfs_->printPath();
}

State CloberPlanner::initiate(const std::vector<agv::Planner::Start>& starts,
  agv::Planner::Goal input_goal,
  agv::Planner::Options options) const
{
  using InternalState = ScheduledDifferentialDriveExpander::InternalState;

  State state{
    Conditions{
      starts,
      std::move(input_goal),
      std::move(options)
    },
    Issues{},
    std::nullopt,
    rmf_utils::make_derived_impl<State::Internal, InternalState>()
  };

  auto& internal = static_cast<InternalState&>(*state.internal);
  const auto& goal = state.conditions.goal;

  ScheduledDifferentialDriveExpander expander{
    state.internal.get(),
    state.issues,
    _supergraph,
    DifferentialDriveHeuristicAdapter{
      _cache->get(),
    _supergraph,
    goal.waypoint(),
      rmf_utils::pointer_to_opt(goal.orientation())
    },
    goal,
    state.conditions.options
  };

  for (const auto& start : starts)
  {
    if (auto node = expander.make_start_node(start))
      internal.queue.push(node);
  }

  if (internal.queue.empty())
  {
    state.issues.disconnected = true;
  }
  else
  {
    const auto& top = internal.queue.top();
    state.ideal_cost = top->get_total_cost_estimate();
  }

  return state;

}

//==============================================================================
std::optional<PlanData> CloberPlanner::clober_plan(State& state,
  std::string target_robot_id,
  std::string target_start,
  std::string target_end,
  std::vector<std::string> target_path,
  std::string enemy_robot_id,
  std::string enemy_start,
  std::size_t enemy_startidx,
  std::string enemy_end,
  std::vector<std::string> enemy_path) const
  {

  std::lock_guard<std::mutex> lock(plan_mutex_);

  RobotInfo target;
  target.robotId_ = target_robot_id;
  target.start_ = target_start;
  target.end_ = target_end;
  target.path_ = target_path;

  std::cout << "CloberPlanner target_robot_id : " << target_robot_id <<
          std::endl;
  // std::cout << "CloberPlanner target_start : " << target_start <<std::endl;
  // std::cout << "CloberPlanner target_end : " << target_end <<std::endl;

  std::cout << "target path : ";
  for (int i = 0; i < target.path_.size(); i++)
  {
    std::cout << target.path_[i] << ", ";
  }
  std::cout << std::endl;

  RobotInfo enemy;
  enemy.robotId_ = enemy_robot_id;
  enemy.start_ = enemy_start;
  enemy.startIdx_ = enemy_startidx;
  enemy.end_ = enemy_end;
  enemy.path_ = enemy_path;

  std::cout << "CloberPlanner enemy_robot_id : " << enemy_robot_id <<std::endl;
  // std::cout << "CloberPlanner enemy_start : " << enemy_start <<std::endl;
  // std::cout << "CloberPlanner enemy_startidx : " << enemy_startidx <<std::endl;
  // std::cout << "CloberPlanner enemy_end : " << enemy_end <<std::endl;

  std::cout << "enemy path : ";
  for (int i = 0; i < enemy.path_.size(); i++)
  {
    std::cout << enemy.path_[i] << ", ";
  }
  std::cout << std::endl;


  std::vector<std::string> name_path = FindNewPath(target, enemy);
  std::vector<std::size_t> idpath;

  if (name_path.size() > 0)
  {
    if (convertNametoIdPath(name_path, idpath))
    {
      std::cout << "graph name to id 로  잘 변환 되었음" << std::endl;
    }
    else
    {
      idpath.clear();
      std::cout << "graph name to id 로 변환 안됨.. error" << std::endl;
    }
  }

  std::vector<Route> routes;
  Trajectory tj;
  std::vector<agv::Plan::Waypoint> waypoints;

  if (idpath.size() > 0)
  {
    // std::cout << "CloberPlanner FindNewPath 결과" << std::endl;

    auto node = std::make_shared<rclcpp::Node>("clober_planner");

    std::chrono::steady_clock::time_point t_start =
      std::chrono::steady_clock::time_point(std::chrono::nanoseconds(node
        ->now().nanoseconds()));

    for (int i = 0; i < idpath.size(); i++)
    {
      std::size_t s = idpath[i];

      const Eigen::Vector2d p = _config.graph().get_waypoint(s).get_location();

      float yaw;
      if( i < idpath.size()-1 ){
        std::size_t n = idpath[i+1];
        const Eigen::Vector2d vec_n = _config.graph().get_waypoint(n).get_location();
    
        yaw = atan2((vec_n[1]-p[1]),(vec_n[0]-p[0]));
      }else{
        yaw = 0.0;
      }

      rmf_traffic::Duration du = rmf_traffic::time::from_seconds(100.0);
      rmf_traffic::Time tt = t_start + du;

      std::time_t tt_print = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now() +
        (tt - std::chrono::steady_clock::now()));
      std::tm* tm = std::localtime(&tt_print);
      char buff[32];
      // Format: Mo, 15.06.2009 20:20:00
      std::strftime(buff, 32, "%a, %d.%m.%Y %H:%M:%S", tm);

      tj.insert(
        tt,
        Eigen::Vector3d{p[0], p[1], yaw},
        Eigen::Vector3d::Zero()
      );

      rmf_utils::optional<std::size_t> graph_index(s);
      Graph::Lane::EventPtr event;

      std::vector<std::size_t> app_lanes = _config.graph().lanes_from(s);

      waypoints.emplace_back(Plan::Waypoint::Implementation::make(
          Eigen::Vector3d{p[0], p[1], yaw}, tt, s, app_lanes, s, s, event
      ));

      t_start = tt;
    }

    Route route("L1", tj);
    routes.push_back(route);

    agv::Planner::Start start = state.conditions.starts[0];
    double cost;

    return PlanData{std::move(routes), std::move(waypoints), std::move(start),
      cost};
  }
  else
  {
    std::cout << "CloberPlanner Negotiation MIP Plan 결과를 찾지 못함. 기본 plan 호출 " <<
            std::endl;
    return plan(state);
  }

}

//==============================================================================
bool CloberPlanner::convertNametoIdPath(const std::vector<std::string> path,
  std::vector<std::size_t>& idpath) const
{

  idpath.clear();

  for (int i = 0; i < path.size(); i++)
  {
    auto it = name_idGraph_.find(path[i]);
    if (it == name_idGraph_.end())
    {
      std::cout <<"convertNametoIdPath mapping failed"<<std::endl;
      return false;
    }
    else
    {
      // std::cout << "mapping name : " << it->first << ", id : " << it->second
      //           << std::endl;
      idpath.push_back(it->second);
    }
  }

  // std::cout <<"convertNametoIdPath path size : " << path.size() <<std::endl;
  // std::cout <<"convertNametoIdPath idpath size : " << idpath.size() <<std::endl;


  return true;
}


//==============================================================================
std::optional<PlanData> CloberPlanner::plan(State& state) const
{
  std::lock_guard<std::mutex> lock(plan_mutex_);

  std::string startStr;
  std::string endStr;
  std::size_t startUint;
  std::size_t endUint;

  for (int i = 0; i < state.conditions.starts.size(); i++)
  {
    startStr = std::to_string(state.conditions.starts[i].waypoint());
    startUint = state.conditions.starts[i].waypoint();
  }

  endStr = std::to_string(state.conditions.goal.waypoint());
  endUint = state.conditions.goal.waypoint();

  auto startIt = id_nameGraph_.find(startUint);
  auto endIt = id_nameGraph_.find(endUint);
  
  // bfs 중 첫번째 경로
  std::vector<std::vector<std::string>> candidates;
  pbfs_->bfs_paths(startStr, endStr, candidates);
  // pbfs_->printPath();

  std::vector<std::string> path;
  if (candidates.size() > 0)
    path = candidates[0];

  if (path.size() == 1)
  {
    path.push_back(path[0]);
  }

  std::vector<Route> routes;
  Trajectory tj;
  std::vector<agv::Plan::Waypoint> waypoints;

  auto node = std::make_shared<rclcpp::Node>("clober_planner");

  std::chrono::steady_clock::time_point t_start =
    std::chrono::steady_clock::time_point(std::chrono::nanoseconds(node->
      now().nanoseconds()));
  // std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();

  for (int i = 0; i < path.size(); i++)
  {
    std::istringstream ss(path[i]);
    std::size_t s;
    ss >> s;

    const Eigen::Vector2d p = _config.graph().get_waypoint(s).get_location();

    float yaw;
    if( i < path.size()-1 ){
      std::istringstream nn(path[i+1]);
      std::size_t n;
      nn >> n;
      const Eigen::Vector2d vec_n = _config.graph().get_waypoint(n).get_location();
  
      yaw = atan2((vec_n[1]-p[1]),(vec_n[0]-p[0]));
    }else{
      yaw = 0.0;
    }

    rmf_traffic::Duration du = rmf_traffic::time::from_seconds(100.0);
    rmf_traffic::Time tt = t_start + du;

    std::time_t tt_print = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now() +
      (tt - std::chrono::steady_clock::now()));
    std::tm* tm = std::localtime(&tt_print);
    char buff[32];
    // Format: Mo, 15.06.2009 20:20:00
    std::strftime(buff, 32, "%a, %d.%m.%Y %H:%M:%S", tm);

    tj.insert(
      tt,
      Eigen::Vector3d{p[0], p[1], yaw},
      Eigen::Vector3d::Zero()
    );

    rmf_utils::optional<std::size_t> graph_index(s);
    Graph::Lane::EventPtr event;

    std::vector<std::size_t> app_lanes = _config.graph().lanes_from(s);

    waypoints.emplace_back(Plan::Waypoint::Implementation::make(
        Eigen::Vector3d{p[0], p[1], yaw}, tt, s, app_lanes, s, s, event
    ));

    t_start = tt;
  }

  Route route("L1", tj);
  routes.push_back(route);

  agv::Planner::Start start = state.conditions.starts[0];
  double cost;

  return PlanData{
    std::move(routes),
    std::move(waypoints),
    std::move(start),
    cost
  };
}


std::vector<std::string> CloberPlanner::FindNewPath(RobotInfo target,
  RobotInfo enemy) const
{

  // std::cout << "target robot: " << target.robotId_ << ", enemy robot: " <<
  //         enemy.robotId_ << std::endl;

  // 등록된 로봇셋 제거
  scheduler_->ClearRobotSet();

  // 타겟 로봇 등록
  scheduler_->SetRobot(target.robotId_, target.start_, target.end_);

  // 상대 로봇 등록
  scheduler_->SetRobot(enemy.robotId_, enemy.start_, enemy.end_);

  // timestep 변수 등록
  scheduler_->MakeStepVariables(target.robotId_);
  scheduler_->MakeStepVariables(enemy.robotId_);

  // init 변수 등록
  scheduler_->MakeInitVariables(target.robotId_);
  scheduler_->MakeInitVariables(enemy.robotId_);

  // arc collision 조건 등록
  scheduler_->MakeArcCollisionCapacity();
  // node collision 조건 등록
  scheduler_->MakeNodeCollisionCapacity();

  // flow conservation 조건 등록
  scheduler_->MakeFlowConservation(target.robotId_);
  scheduler_->MakeFlowConservation(enemy.robotId_);

  // occupied 조건 등록
  scheduler_->MakeOccupiedPath(enemy.robotId_, enemy.startIdx_, enemy.path_);

  // objective 함수 등록
  scheduler_->SetObjective(target.robotId_, target.end_);

  // solver 호출
  std::vector<std::string> newPath;
  newPath = scheduler_->Solve(target.robotId_);


  if (newPath.size() > 0)
  {
    // std::cout <<"우회경로 찾음.. 로봇 : "<<target.robotId_<<", 경로 사이즈 : "<<
            // newPath.size()<<std::endl;
    std::cout <<" 새로운 우회경로 찾음 "<<std::endl;
    for (int i = 0; i < newPath.size(); i++)
    {
      std::cout <<newPath[i]<<", ";
    }
    std::cout <<std::endl;
  }
  else
  {
    std::cout <<"solver 해가 없음"<<std::endl;
  }

  return newPath;
}


std::vector<schedule::Itinerary> CloberPlanner::rollout(
  const Duration span,
  const Issues::BlockedNodes& nodes,
  const Planner::Goal& goal,
  const Planner::Options& options,
  std::optional<std::size_t> max_rollouts) const
{

  std::cout << "CloberPlanner rollout" << std::endl;
}

const Planner::Configuration& CloberPlanner::get_configuration() const
{
  std::cout << "CloberPlanner get_configuration" << std::endl;
  return _config;
}

auto CloberPlanner::debug_begin(
  const std::vector<Planner::Start>& starts,
  Planner::Goal goal,
  Planner::Options options) const -> std::unique_ptr<Debugger>
{
  std::cout << "CloberPlanner debug_begin" << std::endl;

}

std::optional<PlanData> CloberPlanner::debug_step(Debugger& debugger) const
{
  std::cout << "CloberPlanner debug_step" << std::endl;

}


}  // namespace planning
}  // namespace agv
}  // namespace rmf_traffic

#endif