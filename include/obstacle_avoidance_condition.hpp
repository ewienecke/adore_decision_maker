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

#include <cmath>

#include <deque>
#include <iostream>
#include <limits>
#include <type_traits>
#include <vector>

#include <algorithm>
#include <optional>

#include "adore_math/spline.h"

#include "dynamics/integration.hpp"
#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/vehicle_state.hpp"
#include "planning/idm.hpp"
#include "planning/path_shift.hpp"
#include <eigen3/Eigen/Dense>


namespace adore
{

namespace planner
{

std::optional<PathShiftCandidate>
find_static_blocker_on_route( const map::Route& route,
                              const dynamics::VehicleStateDynamic& ego,
                              const dynamics::TrafficParticipantSet& traffic,
                              const dynamics::PhysicalVehicleParameters& ego_params,
                              const PathShiftParams& cfg );


bool
has_oncoming_conflict( const map::Route& route,
                       const dynamics::VehicleStateDynamic& ego,
                       const dynamics::TrafficParticipantSet& traffic,
                       const PathShiftCandidate& blocker,
                       const PathShiftParams& cfg );

bool
has_predicted_oncoming_conflict( const map::Route& route,
                                 const dynamics::VehicleStateDynamic& ego,
                                 const dynamics::TrafficParticipantSet& traffic,
                                 const PathShiftCandidate& blocker,
                                 const PathShiftParams& cfg,
                                 const dynamics::PhysicalVehicleParameters& ego_vehicle_params,
                                 double ego_target_speed );

} // namespace planner

} // namespace adore