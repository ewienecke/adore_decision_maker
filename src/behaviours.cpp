/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "behaviours.hpp"

#include "planning/planning_helpers.hpp" // your existing helpers


namespace
{
enum class AvoidanceState
{
  FOLLOW_ROUTE,
  WAIT_FOR_ONCOMING,
  AVOID_STATIC_OBJECT
};

struct AvoidanceContext
{
  AvoidanceState                 state = AvoidanceState::FOLLOW_ROUTE;
  bool                           has_candidate = false;
  adore::planner::PathShiftCandidate candidate;
};

AvoidanceContext g_avoidance_ctx;
} // namespace

namespace adore::behaviours
{
Decision
emergency_stop( const Domain& domain, PlanningParams& planning_tools )
{

  Decision             out;
  dynamics::Trajectory emergency_stop_trajectory;
  if( domain.vehicle_state )
    emergency_stop_trajectory.states.push_back( domain.vehicle_state.value() );
  emergency_stop_trajectory.label = "Emergency Stop";
  out.trajectory                  = std::move( emergency_stop_trajectory );
  out.traffic_participant         = make_default_participant( domain, planning_tools );
  return out;
}

Decision
standstill( const Domain& domain, PlanningParams& planning_tools )
{

  Decision             out;
  dynamics::Trajectory standstill_trajectory;
  standstill_trajectory.label = "Standstill";
  if( domain.vehicle_state )
    standstill_trajectory.states.push_back( domain.vehicle_state.value() );
  out.trajectory          = std::move( standstill_trajectory );
  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
follow_reference( const Domain& domain, PlanningParams& planning_tools )
{
  Decision out;
  out.trajectory        = *domain.reference_trajectory;
  out.trajectory->label = "Follow Reference";

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

// Decision
// follow_route( const Domain& domain, PlanningParams& planning_tools )
// {
//   Decision out;
//   auto     route_with_signal = domain.route.value();
//   for( auto& p : route_with_signal.reference_line )
//   {
//     if( std::any_of( domain.traffic_signals.begin(), domain.traffic_signals.end(), [&]( const auto& s ) {
//           return adore::math::distance_2d( s.second, p.second ) < 3.0 && s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
//         } ) )
//       p.second.max_speed = 0;
//   }
//   auto traj = planning_tools.planner.plan_route_trajectory( route_with_signal, *domain.vehicle_state, domain.traffic_participants );
//   traj.adjust_start_time( domain.vehicle_state->time );
//   traj.label              = "Follow Route";
//   out.trajectory          = std::move( traj );
//   out.traffic_participant = make_default_participant( domain, planning_tools );
//   return out;
// }

Decision
follow_route( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out;
  auto route_with_signal = domain.route.value();
  const auto& ego = *domain.vehicle_state;

  // Bestehende Ampellogik beibehalten
  for( auto& p : route_with_signal.reference_line )
  {
    if( std::any_of( domain.traffic_signals.begin(),
                     domain.traffic_signals.end(),
                     [&]( const auto& s ) {
                       return adore::math::distance_2d( s.second, p.second ) < 3.0 &&
                              s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
                     } ) )
    {
      p.second.max_speed = 0.0;
    }
  }

  const double ego_s = route_with_signal.get_s( ego );

  // Single-blocker bleibt vorerst noch als einfacher Trigger erhalten.
  auto blocker = planner::find_static_blocker_on_route(
      route_with_signal,
      ego,
      domain.traffic_participants,
      planning_tools.vehicle_model->params,
      planning_tools.path_shift );

  // Für Pfad, Stopppunkt und Manöverende aber immer die gesamte aktuelle Objektmenge verwenden.
  auto obstacles = planner::collect_static_route_obstacles(
      route_with_signal,
      ego,
      domain.traffic_participants,
      planning_tools.vehicle_model->params,
      planning_tools.path_shift );

  // -------------------------
  // State transitions
  // -------------------------
  switch( g_avoidance_ctx.state )
  {
    case AvoidanceState::FOLLOW_ROUTE:
    {
      // maneuver only if a blocker is detected
      if( blocker.has_value() )
      {
        RCLCPP_INFO( rclcpp::get_logger( "DecisionMaker" ), "Blocker detected on route, checking for oncoming traffic..." );
        // oncoming traffic -> wait
        if( planner::has_predicted_oncoming_conflict(
            route_with_signal,
            ego,
            domain.traffic_participants,
            *blocker,
            planning_tools.path_shift,
            planning_tools.vehicle_model->params,
            planning_tools.path_shift.target_speed ) )
        {
          RCLCPP_INFO( rclcpp::get_logger( "DecisionMaker" ), "Oncoming conflict detected, waiting..." );
          g_avoidance_ctx.state = AvoidanceState::WAIT_FOR_ONCOMING;
          g_avoidance_ctx.has_candidate = true;
          g_avoidance_ctx.candidate = *blocker;
        }
        else
        {
          RCLCPP_INFO( rclcpp::get_logger( "DecisionMaker" ), "Static blocker detected -> performing path shift maneuver" );
          // no conflict -> perform path shift maneuver
          g_avoidance_ctx.state = AvoidanceState::AVOID_STATIC_OBJECT;
          g_avoidance_ctx.has_candidate = true;
          g_avoidance_ctx.candidate = *blocker;
        }
      }
      break;
    }

    case AvoidanceState::WAIT_FOR_ONCOMING:
    {
      if( !blocker.has_value() )
      {
        RCLCPP_INFO( rclcpp::get_logger( "DecisionMaker" ), "No more blocker detected, resuming normal route following" );
        g_avoidance_ctx.state = AvoidanceState::FOLLOW_ROUTE;
        g_avoidance_ctx.has_candidate = false;
      }
      else
      {
        g_avoidance_ctx.candidate = *blocker;
        g_avoidance_ctx.has_candidate = true;

        const bool oncoming_conflict = planner::has_predicted_oncoming_conflict(
          route_with_signal,
          ego,
          domain.traffic_participants,
          *blocker,
          planning_tools.path_shift,
          planning_tools.vehicle_model->params,
          planning_tools.path_shift.target_speed );

        if( !oncoming_conflict )
        {
          RCLCPP_INFO( rclcpp::get_logger( "DecisionMaker" ), "Oncoming conflict cleared, performing path shift maneuver" );
          g_avoidance_ctx.state = AvoidanceState::AVOID_STATIC_OBJECT;
        }
      }
      break;
    }

    case AvoidanceState::AVOID_STATIC_OBJECT:
    {
      const bool oncoming_conflict = planner::has_predicted_oncoming_conflict(
      route_with_signal,
      ego,
      domain.traffic_participants,
      g_avoidance_ctx.candidate,
      planning_tools.path_shift,
      planning_tools.vehicle_model->params,
      planning_tools.path_shift.target_speed );

        if( oncoming_conflict )
        {
          RCLCPP_INFO( rclcpp::get_logger( "DecisionMaker" ), "Oncoming conflict detected during avoidance, switching to wait" );
          g_avoidance_ctx.state = AvoidanceState::WAIT_FOR_ONCOMING;
          break;
        }
        
      if( !g_avoidance_ctx.has_candidate )
      {
        RCLCPP_INFO( rclcpp::get_logger( "DecisionMaker" ), "No candidate available, resuming normal route following" );
        g_avoidance_ctx.state = AvoidanceState::FOLLOW_ROUTE;
        break;
      }

      if( obstacles.empty() )
      {
        RCLCPP_INFO( rclcpp::get_logger( "DecisionMaker" ), "No more relevant obstacles, resuming normal route following" );
        g_avoidance_ctx.state = AvoidanceState::FOLLOW_ROUTE;
        g_avoidance_ctx.has_candidate = false;
        break;
      }

      // Ende des Manövers über das letzte aktuell relevante Objekt bestimmen
      const auto& last_obstacle = obstacles.back();
      const double dynamic_return_length =
          std::max( planning_tools.path_shift.return_length,
                    8.0 * last_obstacle.required_shift );
      const double active_end_s = last_obstacle.occ_end_s + dynamic_return_length;

      if( ego_s > active_end_s + 2.0 )
      {
        RCLCPP_INFO( rclcpp::get_logger( "DecisionMaker" ), "No more relevant obstacles, resuming normal route following" );
        g_avoidance_ctx.state = AvoidanceState::FOLLOW_ROUTE;
        g_avoidance_ctx.has_candidate = false;
      }
      break;
    }
    
  }

  // -------------------------
  // WAIT state
  // -------------------------
  if( g_avoidance_ctx.state == AvoidanceState::WAIT_FOR_ONCOMING &&
    g_avoidance_ctx.has_candidate )
  {
    auto wait_route = route_with_signal;

    for( auto& [s, mp] : wait_route.reference_line )
    {
      if( s >= g_avoidance_ctx.candidate.stop_s )
        mp.max_speed = 0.0;
    }

    auto traj = planning_tools.planner.plan_route_trajectory(
        wait_route,
        ego,
        domain.traffic_participants );

    traj.adjust_start_time( ego.time );
    traj.label = "Wait For Oncoming";

    out.trajectory = std::move( traj );
    out.traffic_participant = make_default_participant( domain, planning_tools );
    return out;
  }

  // -------------------------
  // AVOID state
  // -------------------------
    if( g_avoidance_ctx.state == AvoidanceState::AVOID_STATIC_OBJECT &&
    g_avoidance_ctx.has_candidate )
    {
      if( !obstacles.empty() )
      {
        auto shifted_points = planner::build_shifted_path_samples_from_obstacles(
            route_with_signal,
            ego_s,
            planning_tools.path_shift.lookahead_length,
            0.25,
            obstacles,
            planning_tools.path_shift );

        if( shifted_points.size() >= 2 )
        {
          auto traj = planner::waypoints_to_trajectory(
              ego,
              shifted_points,
              domain.traffic_participants,
              *planning_tools.vehicle_model,
              planning_tools.path_shift.target_speed );

          traj.adjust_start_time( ego.time );
          traj.label = "Path Shift Envelope";

          out.trajectory = std::move( traj );
          out.traffic_participant = make_default_participant( domain, planning_tools );
          return out;
        }
      }

    // Falls keine relevanten Objekte mehr vorhanden sind oder die Pfaderzeugung fehlschlägt:
    // zurück auf Follow Route.
    g_avoidance_ctx.state = AvoidanceState::FOLLOW_ROUTE;
    g_avoidance_ctx.has_candidate = false;
  }

  // -------------------------
  // Normal Follow Route
  // -------------------------
  auto traj = planning_tools.planner.plan_route_trajectory(
      route_with_signal,
      ego,
      domain.traffic_participants );

  traj.adjust_start_time( ego.time );
  traj.label = "Follow Route";

  out.trajectory = std::move( traj );
  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
waiting_for_assistance( const Domain& domain, PlanningParams& planning_tools )
{

  // if we have no waypoints, do nothing
  Decision out          = minimum_risk( domain, planning_tools );
  out.trajectory->label = "Waiting for Waypoints";

  if( domain.waypoints.has_value() && domain.waypoints->waypoints.size() > 1 )
  {
    // if we have waypoints, create trajectory to send
    dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, domain.waypoints->waypoints,
                                                                        domain.traffic_participants, *planning_tools.vehicle_model );
    trajectory.label                = "Suggested Trajectory";
    trajectory.adjust_start_time( domain.vehicle_state->time );
    out.trajectory_suggestion = std::move( trajectory );
    out.trajectory->label     = "Waiting for Confirmation";
  }

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
follow_assistance( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out;

  dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, domain.waypoints->waypoints,
                                                                      domain.traffic_participants, *planning_tools.vehicle_model );
  trajectory.label                = "Follow Assistance";
  trajectory.adjust_start_time( domain.vehicle_state->time );
  out.trajectory = std::move( trajectory );

  out.traffic_participant = make_default_participant( domain, planning_tools );
  out.assistance_request  = false;
  return out;
}

Decision
safety_corridor( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out; // calculate trajectory getting out of safety corridor
  auto     right_forward_points = planner::filter_points_in_front( domain.safety_corridor->right_border, *domain.vehicle_state );
  auto     safety_waypoints     = planner::shift_points_right( right_forward_points, planning_tools.vehicle_model->params.body_width );
  double   target_speed         = planner::is_point_to_right_of_line( *domain.vehicle_state, right_forward_points ) ? 0 : 2.0;

  auto planned_trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, safety_waypoints, domain.traffic_participants,
                                                              *planning_tools.vehicle_model, target_speed );

  planned_trajectory = planning_tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );

  planned_trajectory.label = "Safety Corridor";
  out.trajectory           = std::move( planned_trajectory );

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
request_assistance( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out            = minimum_risk( domain, planning_tools );
  out.assistance_request  = true;
  out.trajectory->label   = "Request Assistance";
  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
minimum_risk( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out;

  double state_s            = domain.route->get_s( *domain.vehicle_state );
  auto   cut_route          = domain.route->get_shortened_route( state_s, 100.0 );
  auto   planned_trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, cut_route, domain.traffic_participants,
                                                                *planning_tools.vehicle_model, 0.0 /* target_speed */ );

  // planned_trajectory = planning_tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );
  if( planned_trajectory.states.size() < 2 )
  {
    out = standstill( domain, planning_tools );
  }
  planned_trajectory.label = "Minimum Risk Maneuver";
  out.trajectory           = std::move( planned_trajectory );

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

dynamics::TrafficParticipant
make_default_participant( const Domain& domain, const PlanningParams& planning_tools )
{
  dynamics::TrafficParticipant participant;
  if( domain.vehicle_state )
    participant.state = domain.vehicle_state.value();
  if( domain.route )
  {
    participant.goal_point = domain.route->destination;
    participant.route      = domain.route.value();
  }
  participant.id                  = planning_tools.v2x_id;
  participant.v2x_id              = planning_tools.v2x_id;
  participant.classification      = dynamics::CAR;
  participant.physical_parameters = planning_tools.vehicle_model->params;
  return participant;
}

} // namespace adore::behaviours
