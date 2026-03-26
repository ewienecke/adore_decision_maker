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

#pragma once
#include <optional>

#include "adore_dynamics_conversions.hpp"

#include "dynamics/comfort_settings.hpp"
#include "planning/trajectory_planner.hpp"

namespace adore
{

struct Decision
{
  std::optional<dynamics::Trajectory>         trajectory;
  std::optional<dynamics::TrafficParticipant> traffic_participant;
  std::optional<dynamics::Trajectory>         trajectory_suggestion;
  std::optional<bool>                         assistance_request;
};

struct PathShiftParams
{
  double min_object_ahead        = 6.0;
  double max_object_ahead        = 35.0;
  double max_object_speed        = 0.5;   // [m/s] maximum speed of objects to consider for path shifting
  double static_clearance        = 0.4;   // [m] additional clearance to the object when calculating required shift 
  double front_clearance         = 4.0;   // [m] safty distance in front of the object when full shift should be reached
  double rear_clearance          = 3.0;   // [m] safty distance behind the object until the shift should be held
  double approach_length         = 15.0;  // [m] length of the approach phase before reaching the object; higher value leads to earlier and smoother shifts
  double return_length           = 18.0;  
  double target_speed            = 3.0;   // [m/s] target speed during the shift maneuver
  double lookahead_length        = 50.0;  // [m] length of the path ahead to consider for shift decisions
  double max_shift_left          = 2.0;
  double route_overlap_slack     = 0.5;   // [m] tolerance for considering an object as overlapping the route
  double oncoming_front_buffer   = 20.0;  // [m] buffer for oncoming traffic in front
  double oncoming_rear_buffer    = 5.0;   // [m] buffer for oncoming traffic behind
  double min_oncoming_angle_diff = 2.0;   // [rad] minimum angle difference to route for considering a vehicle as oncoming

  double prediction_time_step              = 0.1;  // [s]
  double prediction_time_horizon           = 6.0;  // [s]
  double min_ego_prediction_speed          = 1.0;  // [m/s]
  double min_oncoming_route_speed          = 0.2;  // [m/s]
  double max_stationary_conflict_route_speed = 0.3; // [m/s]
  double oncoming_vehicle_s_margin         = 1.0;  // [m]
  double static_oncoming_s_margin          = 1.0;  // [m]
  double ego_vehicle_s_margin              = 1.0;  // [m]
};

struct PlanningParams
{
  planner::TrajectoryPlanner                      planner;
  std::shared_ptr<dynamics::PhysicalVehicleModel> vehicle_model;
  std::shared_ptr<dynamics::ComfortSettings>      comfort_settings;
  std::map<std::string, double>                   planner_settings;
  int                                             v2x_id = 0;

  PathShiftParams                                  path_shift;
};

// define condition parameters
struct ConditionParams
{
  // reference trajectory
  size_t min_ref_traj_size = 5;
  double max_ref_traj_age  = 1.0; // [s]
  size_t min_route_length  = 20;  // [m]
  double gps_sigma_ok      = 1.0; // [m]s
};

struct DomainParams
{
  double max_participant_age = 1.0;
  int    v2x_id              = 0;
};

struct InTopics
{
  std::string state                           = "vehicle_state_dynamic";
  std::string route                           = "route";
  std::string safety_corridor                 = "safety_corridor";
  std::string suggested_trajectory            = "suggested_trajectory";
  std::string reference_trajectory            = "reference_trajectory";
  std::string sensor_participants             = "traffic_participants";
  std::string infrastructure_participants     = "/planned_traffic";
  std::string traffic_signals                 = "traffic_signals";
  std::string waypoints                       = "remote_operation_waypoints";
  std::string suggested_trajectory_acceptance = "suggested_trajectory_accepted";
  std::string caution_zones                   = "caution_zones";
  std::string assistance_request              = "assistance_request";
};

struct OutTopics
{
  std::string trajectory_decision   = "trajectory_decision";
  std::string trajectory_suggestion = "trajectory_suggestion";
  std::string assistance_request    = "assistance_request";
  std::string traffic_participant   = "traffic_participant";
};

struct DecisionParams
{
  double          run_delta_time = 0.1; // seconds, how often to run the decision maker
  bool            debug          = false;
  ConditionParams condition_params;
  PlanningParams  planning_params;
  DomainParams    domain_params;
  InTopics        in_topics;
  OutTopics       out_topics;
};

inline DecisionParams
load_params( rclcpp::Node& node )
{
  DecisionParams params;

  auto& condition_params = params.condition_params;
  auto& planning_params  = params.planning_params;
  auto& domain_params    = params.domain_params;
  auto& in_topics        = params.in_topics;
  auto& out_topics       = params.out_topics;

  params.run_delta_time = node.declare_parameter( "run_delta_time", params.run_delta_time );
  params.debug          = node.declare_parameter( "debug", params.debug );

  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- Conditions -------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  condition_params.gps_sigma_ok      = node.declare_parameter( "gps_sigma_ok", condition_params.gps_sigma_ok );
  condition_params.min_ref_traj_size = node.declare_parameter( "min_ref_traj_size", 20 );
  condition_params.max_ref_traj_age  = node.declare_parameter( "max_ref_traj_age", condition_params.max_ref_traj_age );
  condition_params.min_route_length  = node.declare_parameter( "min_route_length", 10 );

  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- Planning ---------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  std::vector<std::string> keys   = node.declare_parameter( "planner_settings_keys", std::vector<std::string>{} );
  std::vector<double>      values = node.declare_parameter( "planner_settings_values", std::vector<double>{} );

  if( keys.size() == values.size() )
  {
    for( size_t i = 0; i < keys.size(); ++i )
    {
      planning_params.planner_settings[keys[i]] = values[i];
    }
  }

  std::string vehicle_model_file = node.declare_parameter( "vehicle_model_file", "" );

  planning_params.vehicle_model    = std::make_shared<dynamics::PhysicalVehicleModel>( vehicle_model_file, false );
  planning_params.comfort_settings = std::make_shared<dynamics::ComfortSettings>(); // default value comfort settings

  planning_params.planner.set_vehicle_parameters( planning_params.vehicle_model->params );
  planning_params.planner.set_comfort_settings( planning_params.comfort_settings );
  planning_params.planner.set_parameters( planning_params.planner_settings );

  planning_params.v2x_id = node.declare_parameter( "v2x_id", planning_params.v2x_id );


  // Path shift parameters for static obstacle avoidance
  // overwrite default values with parameters from the parameter server if available

    planning_params.path_shift.min_object_ahead =
      node.declare_parameter( "path_shift.min_object_ahead",
                              planning_params.path_shift.min_object_ahead );
  planning_params.path_shift.max_object_ahead =
      node.declare_parameter( "path_shift.max_object_ahead",
                              planning_params.path_shift.max_object_ahead );
  planning_params.path_shift.max_object_speed =
      node.declare_parameter( "path_shift.max_object_speed",
                              planning_params.path_shift.max_object_speed );
  planning_params.path_shift.static_clearance =
      node.declare_parameter( "path_shift.static_clearance",
                              planning_params.path_shift.static_clearance );
  planning_params.path_shift.front_clearance =
      node.declare_parameter( "path_shift.front_clearance",
                              planning_params.path_shift.front_clearance );
  planning_params.path_shift.rear_clearance =
      node.declare_parameter( "path_shift.rear_clearance",
                              planning_params.path_shift.rear_clearance );
  planning_params.path_shift.approach_length =
      node.declare_parameter( "path_shift.approach_length",
                              planning_params.path_shift.approach_length );
  planning_params.path_shift.return_length =
      node.declare_parameter( "path_shift.return_length",
                              planning_params.path_shift.return_length );
  planning_params.path_shift.target_speed =
      node.declare_parameter( "path_shift.target_speed",
                              planning_params.path_shift.target_speed );
  planning_params.path_shift.lookahead_length =
      node.declare_parameter( "path_shift.lookahead_length",
                              planning_params.path_shift.lookahead_length );
  planning_params.path_shift.max_shift_left =
      node.declare_parameter( "path_shift.max_shift_left",
                              planning_params.path_shift.max_shift_left );
  planning_params.path_shift.route_overlap_slack =
      node.declare_parameter( "path_shift.route_overlap_slack",
                              planning_params.path_shift.route_overlap_slack );
  planning_params.path_shift.oncoming_front_buffer =
      node.declare_parameter( "path_shift.oncoming_front_buffer",
                              planning_params.path_shift.oncoming_front_buffer );
  planning_params.path_shift.oncoming_rear_buffer =
      node.declare_parameter( "path_shift.oncoming_rear_buffer",
                              planning_params.path_shift.oncoming_rear_buffer );
  planning_params.path_shift.min_oncoming_angle_diff =
      node.declare_parameter( "path_shift.min_oncoming_angle_diff",
                              planning_params.path_shift.min_oncoming_angle_diff );


  planning_params.path_shift.ego_vehicle_s_margin =
    node.declare_parameter( "path_shift.ego_vehicle_s_margin",
                            planning_params.path_shift.ego_vehicle_s_margin );

  planning_params.path_shift.oncoming_vehicle_s_margin =
      node.declare_parameter( "path_shift.oncoming_vehicle_s_margin",
                              planning_params.path_shift.oncoming_vehicle_s_margin );

  planning_params.path_shift.max_stationary_conflict_route_speed =
      node.declare_parameter( "path_shift.max_stationary_conflict_route_speed",
                              planning_params.path_shift.max_stationary_conflict_route_speed );

  planning_params.path_shift.prediction_time_step =
      node.declare_parameter( "path_shift.prediction_time_step",
                              planning_params.path_shift.prediction_time_step );

  planning_params.path_shift.prediction_time_horizon =
      node.declare_parameter( "path_shift.prediction_time_horizon",
                              planning_params.path_shift.prediction_time_horizon );

  planning_params.path_shift.min_ego_prediction_speed =
      node.declare_parameter( "path_shift.min_ego_prediction_speed",
                              planning_params.path_shift.min_ego_prediction_speed );

  planning_params.path_shift.min_oncoming_route_speed =
      node.declare_parameter( "path_shift.min_oncoming_route_speed",
                              planning_params.path_shift.min_oncoming_route_speed );

                              

  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- Domain -----------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  domain_params.max_participant_age = node.declare_parameter( "max_participant_age", domain_params.max_participant_age );
  domain_params.v2x_id              = planning_params.v2x_id;

  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- In Topics --------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  in_topics.state                           = node.declare_parameter( "topic_vehicle_state_dynamic", in_topics.state );
  in_topics.route                           = node.declare_parameter( "topic_route", in_topics.route );
  in_topics.safety_corridor                 = node.declare_parameter( "topic_safety_corridor", in_topics.safety_corridor );
  in_topics.suggested_trajectory            = node.declare_parameter( "topic_suggested_trajectory", in_topics.suggested_trajectory );
  in_topics.reference_trajectory            = node.declare_parameter( "topic_reference_trajectory", in_topics.reference_trajectory );
  in_topics.sensor_participants             = node.declare_parameter( "topic_traffic_participants", in_topics.sensor_participants );
  in_topics.infrastructure_participants     = node.declare_parameter( "topic_infrastructure_participants",
                                                                      in_topics.infrastructure_participants );
  in_topics.traffic_signals                 = node.declare_parameter( "topic_traffic_signals", in_topics.traffic_signals );
  in_topics.waypoints                       = node.declare_parameter( "topic_waypoints", in_topics.waypoints );
  in_topics.suggested_trajectory_acceptance = node.declare_parameter( "topic_suggested_trajectory_accepted",
                                                                      in_topics.suggested_trajectory_acceptance );
  in_topics.caution_zones                   = node.declare_parameter( "topic_caution_zones", in_topics.caution_zones );
  in_topics.assistance_request              = node.declare_parameter( "topic_assistance_request", in_topics.assistance_request );

  // ---------------------------------------------------------------------------------------------------------
  // -------------------------------------------- Out Topics -------------------------------------------------
  // ---------------------------------------------------------------------------------------------------------
  out_topics.trajectory_decision   = node.declare_parameter( "topic_trajectory_decision", out_topics.trajectory_decision );
  out_topics.trajectory_suggestion = node.declare_parameter( "topic_trajectory_suggestion", out_topics.trajectory_suggestion );
  out_topics.assistance_request    = in_topics.assistance_request;
  out_topics.traffic_participant   = node.declare_parameter( "topic_traffic_participant", out_topics.traffic_participant );

  return params;
}

} // namespace adore