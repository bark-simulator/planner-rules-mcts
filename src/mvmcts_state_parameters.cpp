// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "mvmcts_state_parameters.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

bark::models::behavior::MvmctsStateParameters::MvmctsStateParameters(
    const float collision_weight, const float out_of_map_weight, const float potential_weight,
    const float acceleration_weight, const float radial_acceleration_weight,
    const float desired_velocity_weight, const float lane_center_weight,
    const unsigned int reward_vector_size, const float prediction_time_span,
    const float desired_velocity, const unsigned int horizon,
    const float discount_factor, const float goal_reward)
    : COLLISION_WEIGHT(collision_weight),
      OUT_OF_MAP_WEIGHT(out_of_map_weight),
      POTENTIAL_WEIGHT(potential_weight),
      ACCELERATION_WEIGHT(acceleration_weight),
      RADIAL_ACCELERATION_WEIGHT(radial_acceleration_weight),
      DESIRED_VELOCITY_WEIGHT(desired_velocity_weight),
      LANE_CENTER_WEIGHT(lane_center_weight),
      REWARD_VECTOR_SIZE(reward_vector_size),
      PREDICTION_TIME_SPAN(prediction_time_span),
      DESIRED_VELOCITY(desired_velocity),
      HORIZON(horizon),
      DISCOUNT_FACTOR(discount_factor),
      GOAL_REWARD(goal_reward) {}

bark::models::behavior::MvmctsStateParameters::MvmctsStateParameters(
    const commons::ParamsPtr& params)
    : COLLISION_WEIGHT(
          params->GetReal("BehaviorMvmcts::StateParameters::CollisionWeight",
                          "Penalty for collision", 0)),
      OUT_OF_MAP_WEIGHT(
          params->GetReal("BehaviorMvmcts::StateParameters::OutOfMapWeight",
                          "Penalty for leaving the map", -800)),
      POTENTIAL_WEIGHT(
          params->GetReal("BehaviorMvmcts::StateParameters::PotentialWeight",
                          "Weight of the goal potential function", 32.0)),
      ACCELERATION_WEIGHT(
          params->GetReal("BehaviorMvmcts::StateParameters::AccelerationWeight",
                          "Weight for longitudinal accelerations", 0.0)),
      RADIAL_ACCELERATION_WEIGHT(params->GetReal(
          "BehaviorMvmcts::StateParameters::RadialAccelerationWeight",
          "Weight for radial acceleration", 0.0)),
      DESIRED_VELOCITY_WEIGHT(params->GetReal(
          "BehaviorMvmcts::StateParameters::DesiredVelocityWeight",
          "Weight for deviating from the desired velocity", -5.0)),
      LANE_CENTER_WEIGHT(params->GetReal(
          "BehaviorMvmcts::StateParameters::LaneCenterDeviationWeight",
          "Weight for deviating from the lane center", 0.0)),
      REWARD_VECTOR_SIZE(static_cast<unsigned int>(params->GetInt(
          "BehaviorMvmcts::RewardVectorSize", "Size of the reward vector", 1))),
      PREDICTION_TIME_SPAN(
          params->GetReal("BehaviorMvmcts::StateParameters::PredictionTimeSpan",
                          "Time between two consecutive states", 0.3)),
      DESIRED_VELOCITY(
          params->GetReal("BehaviorMvmcts::StateParameters::DesiredVelocity",
                          "Desired driving speed", 10.0)),
      HORIZON(static_cast<unsigned int>(
          params->GetInt("BehaviorMvmcts::StateParameters::Horizon",
                         "Number of steps until termination", 30))),
      DISCOUNT_FACTOR(params->GetReal("BehaviorMvmcts::DiscountFactor",
                                      "Discount factor used in MDP problem",
                                      0.9)),
      GOAL_REWARD(params->GetReal(
          "BehaviorMvmcts::StateParameters::GoalReward",
          "Reward received when reaching the agent's goal.", 0.0)) {}
