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

#include "obstacle_avoidance_condition.hpp"

namespace adore
{

namespace planner
{

std::optional<PathShiftCandidate>
find_static_blocker_on_route( const map::Route& route,
                              const dynamics::VehicleStateDynamic& ego,
                              const dynamics::TrafficParticipantSet& traffic,
                              const dynamics::PhysicalVehicleParameters& ego_params,
                              const PathShiftParams& cfg )
{
  const double ego_s = route.get_s( ego );
  if( !std::isfinite( ego_s ) ) return std::nullopt;

  std::optional<PathShiftCandidate> best;
  double best_s = std::numeric_limits<double>::max();

  for( const auto& [id, participant] : traffic.participants )
  {
    const auto& participant_state = participant.state;

    // first simplification: 
    // only consider objects that are approximately static, i.e. below a certain speed threshold
    if( std::abs( participant_state.vx ) > cfg.max_object_speed ) continue;

    const double obj_s = route.get_s( participant_state );
if( !std::isfinite( obj_s ) ) continue;

const double ahead = obj_s - ego_s;

// Nur zu weit entfernte Objekte verwerfen.
// Ein nahes Objekt darf nicht verschwinden, nur weil Ego schon nahe dran ist.
if( ahead > cfg.max_object_ahead ) continue;

const auto ref_pose = route.get_pose_at_s( obj_s );
const double obj_d =
    signed_lateral_offset( ref_pose, adore::math::Point2d{ participant_state.x, participant_state.y } );

// Falls die gemessenen Maße nicht vorhanden sind, konservative Defaultwerte verwenden
const double obj_w =
    participant.physical_parameters.body_width > 0.1 ? participant.physical_parameters.body_width : 1.8;
const double obj_l =
    participant.physical_parameters.body_length > 0.1 ? participant.physical_parameters.body_length : 4.5;

// Objekt muss nahe genug an der Route liegen, um relevant zu sein
if( std::abs( obj_d ) >
    ( 0.5 * obj_w + 0.5 * ego_params.body_width + cfg.route_overlap_slack ) )
  continue;

// Benötigter Shift
const double required_shift =
    obj_d + 0.5 * obj_w + 0.5 * ego_params.body_width + cfg.static_clearance;

if( required_shift <= 0.1 ) continue;
if( required_shift > cfg.max_shift_left ) continue;

// Objekt erst dann verwerfen, wenn Ego geometrisch wirklich daran vorbei ist
const double dynamic_return_length =
    std::max( cfg.return_length, 8.0 * required_shift );

const double object_keep_until_s =
    obj_s + 0.5 * obj_l + cfg.rear_clearance + dynamic_return_length;

if( ego_s > object_keep_until_s )
  continue;

PathShiftCandidate candidate;
candidate.participant_id = id;
candidate.ego_s          = ego_s;
candidate.object_s       = obj_s;
candidate.object_d       = obj_d;
candidate.object_length  = obj_l;
candidate.object_width   = obj_w;
candidate.shift_left     = required_shift;

// derive maneuver borders based on object length and configured clearances
const double dynamic_approach_length =
    std::max( cfg.approach_length, 8.0 * candidate.shift_left );

candidate.shift_apex_start_s = obj_s - 0.5 * obj_l - cfg.front_clearance;
candidate.shift_start_s      = candidate.shift_apex_start_s - dynamic_approach_length;
candidate.shift_apex_end_s   = obj_s + 0.5 * obj_l + cfg.rear_clearance;
candidate.shift_end_s        = candidate.shift_apex_end_s + dynamic_return_length;

// Stop point for WAIT-State with some additional buffer to the object
candidate.stop_s             = obj_s - 0.5 * obj_l - cfg.front_clearance - 1.0;

// nächster Blocker gewinnt
if( candidate.object_s < best_s )
{
  best = candidate;
  best_s = candidate.object_s;
}
  }

  return best;
}


bool
has_oncoming_conflict( const map::Route& route,
                       const dynamics::VehicleStateDynamic& ego,
                       const dynamics::TrafficParticipantSet& traffic,
                       const PathShiftCandidate& blocker,
                       const PathShiftParams& cfg )
{
  const double ego_s = route.get_s( ego );
  if( !std::isfinite( ego_s ) ) return false;

  for( const auto& [id, participant] : traffic.participants )
  {
    // blocking object itself is no oncoming vehicle
    if( id == blocker.participant_id ) continue;

    const auto& participant_state = participant.state;
    const double participant_s = route.get_s( participant_state );
    if( !std::isfinite( participant_s ) ) continue;

    // only consider vehicles that are within a certain longitudinal distance from the blocking object
    if( participant_s < ego_s - cfg.oncoming_rear_buffer ) continue;
    if( participant_s > blocker.shift_end_s + cfg.oncoming_front_buffer ) continue;

    const auto ref_pose = route.get_pose_at_s( participant_s );
    const double participant_d =
        signed_lateral_offset( ref_pose, adore::math::Point2d{ participant_state.x, participant_state.y } );


    // simplification: oncoming traffic is expected roughtly to the left of our lane
    if( participant_d < 0.0 ) continue;

    const double heading_diff =
        std::abs( adore::math::normalize_angle( participant_state.yaw_angle - ref_pose.yaw ) );

    // when an object is inside the conflict area and its orientation differs significantly from the route direction, 
    // we consider it as a potential oncoming conflict that prevents us from performing the path shift maneuver
    if( heading_diff > cfg.min_oncoming_angle_diff )
      return true;
  }

  return false;
}


bool
has_predicted_oncoming_conflict( const map::Route& route,
                                 const dynamics::VehicleStateDynamic& ego,
                                 const dynamics::TrafficParticipantSet& traffic,
                                 const PathShiftCandidate& blocker,
                                 const PathShiftParams& cfg,
                                 const dynamics::PhysicalVehicleParameters& ego_vehicle_params,
                                 double ego_target_speed )
{
  const double ego_s = route.get_s( ego );
  if( !std::isfinite( ego_s ) ) return false;

  // Konfliktbereich des geplanten Überholmanövers
  const double conflict_start_s = blocker.shift_start_s;
  const double conflict_end_s   = blocker.shift_end_s;

  // Einfaches Ego-Vorhersagemodell entlang der Route
  const double ego_prediction_speed =
      std::max( ego_target_speed, cfg.min_ego_prediction_speed );

  const double ego_half_length =
      0.5 * ego_vehicle_params.body_length + cfg.ego_vehicle_s_margin;

  const double prediction_end_time =
      cfg.prediction_time_horizon;

  for( const auto& [participant_id, traffic_participant] : traffic.participants )
  {
    // Das parkende Blocker-Objekt selbst ist kein Gegenverkehr
    if( participant_id == blocker.participant_id ) continue;

    const auto& participant_state = traffic_participant.state;

    const double participant_s = route.get_s( participant_state );
    if( !std::isfinite( participant_s ) ) continue;

    const auto route_pose_at_participant = route.get_pose_at_s( participant_s );

    const double participant_d =
        signed_lateral_offset(
            route_pose_at_participant,
            adore::math::Point2d{ participant_state.x, participant_state.y } );

    // Einfache Annahme: Gegenverkehr befindet sich links der Route
    if( participant_d < 0.0 ) continue;

    const double heading_difference =
        std::abs(
            adore::math::normalize_angle(
                participant_state.yaw_angle - route_pose_at_participant.yaw ) );

    const double participant_route_speed =
        participant_state.vx *
        std::cos(
            adore::math::normalize_angle(
                participant_state.yaw_angle - route_pose_at_participant.yaw ) );

    const double participant_length =
        traffic_participant.physical_parameters.body_length > 0.1
            ? traffic_participant.physical_parameters.body_length
            : 4.5;

    // --------------------------------------------------
    // Fall A: Fahrzeug steht oder ist fast stationär
    // --------------------------------------------------
    if( std::abs( participant_route_speed ) <= cfg.max_stationary_conflict_route_speed )
    {
      const double participant_half_length =
          0.5 * participant_length + cfg.oncoming_vehicle_s_margin;

      const double occupied_start_s = participant_s - participant_half_length;
      const double occupied_end_s   = participant_s + participant_half_length;

      const bool overlaps_conflict_interval =
          occupied_end_s >= ( conflict_start_s - cfg.oncoming_rear_buffer ) &&
          occupied_start_s <= ( conflict_end_s + cfg.oncoming_front_buffer );

      if( overlaps_conflict_interval )
        return true;

      continue;
    }

    // --------------------------------------------------
    // Fall B: Bewegter Gegenverkehr
    // --------------------------------------------------
    // Nur Teilnehmer betrachten, die grob entgegen der Route orientiert sind
    if( heading_difference <= cfg.min_oncoming_angle_diff ) continue;

    // Nur wirklichen Gegenverkehr betrachten: negative Geschwindigkeit entlang der Route
    if( participant_route_speed >= -cfg.min_oncoming_route_speed ) continue;

    const double participant_half_length =
        0.5 * participant_length + cfg.oncoming_vehicle_s_margin;

    for( double prediction_time = 0.0;
         prediction_time <= prediction_end_time;
         prediction_time += cfg.prediction_time_step )
    {
      // Ego-Vorhersage entlang der Route
      const double predicted_ego_s =
          ego_s + ego_prediction_speed * prediction_time;

      const double predicted_ego_start_s =
          predicted_ego_s - ego_half_length;

      const double predicted_ego_end_s =
          predicted_ego_s + ego_half_length;

      // Ego nur betrachten, wenn es überhaupt im Konfliktbereich ist
      const bool ego_is_in_conflict_region =
          predicted_ego_end_s >= conflict_start_s &&
          predicted_ego_start_s <= conflict_end_s;

      if( !ego_is_in_conflict_region )
        continue;

      // Vorhersage des Gegenverkehrs entlang der Route
      const double predicted_participant_s =
          participant_s + participant_route_speed * prediction_time;

      const double predicted_participant_start_s =
          predicted_participant_s - participant_half_length;

      const double predicted_participant_end_s =
          predicted_participant_s + participant_half_length;

      // Gegenverkehr nur betrachten, wenn er ebenfalls im Konfliktbereich liegt
      const bool participant_is_in_conflict_region =
          predicted_participant_end_s >= ( conflict_start_s - cfg.oncoming_rear_buffer ) &&
          predicted_participant_start_s <= ( conflict_end_s + cfg.oncoming_front_buffer );

      if( !participant_is_in_conflict_region )
        continue;

      // Zentrale Änderung:
      // Konflikt nur dann, wenn Ego und Gegenverkehr zur GLEICHEN ZEIT
      // longitudinal überlappen.
      const bool occupied_intervals_overlap =
          predicted_participant_end_s >= predicted_ego_start_s &&
          predicted_participant_start_s <= predicted_ego_end_s;

      if( occupied_intervals_overlap )
        return true;
    }
  }

  return false;
}


} // namespace planner

} // namespace adore