// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_MVMCTS_STATE_PARAMETERS_HPP_
#define MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_MVMCTS_STATE_PARAMETERS_HPP_

#include "modules/commons/params/setter_params.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::commons::Params;

class MvmctsStateParameters {
 public:
  MvmctsStateParameters(
      const float out_of_map_weight, const float potential_weight,
      const float acceleration_weight, const float radial_acceleration_weight,
      const float desired_velocity_weight, const float lane_center_weight,
      const unsigned int reward_vector_size, const float prediction_time_span,
      const float desired_velocity, const unsigned int horizon,
      const float discount_factor);
  explicit MvmctsStateParameters(const commons::ParamsPtr &params);
  const float OUT_OF_MAP_WEIGHT;
  const float POTENTIAL_WEIGHT;
  const float ACCELERATION_WEIGHT;
  const float RADIAL_ACCELERATION_WEIGHT;
  const float DESIRED_VELOCITY_WEIGHT;
  const float LANE_CENTER_WEIGHT;
  const unsigned int REWARD_VECTOR_SIZE;
  const float PREDICTION_TIME_SPAN;
  const float DESIRED_VELOCITY;
  const unsigned int HORIZON;
  const float DISCOUNT_FACTOR;
};

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_MVMCTS_STATE_PARAMETERS_HPP_
