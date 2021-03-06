// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "rules_mcts_state_parameters.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

bark::models::behavior::RulesMctsStateParameters::RulesMctsStateParameters(
    const double collision_weight, const double out_of_map_weight,
    const double potential_weight, const double acceleration_weight,
    const double radial_acceleration_weight, const double desired_velocity_weight,
    const double lane_center_weight, const unsigned int reward_vector_size,
    const double prediction_time_span, const double desired_velocity,
    const unsigned int horizon, const double discount_factor,
    const double goal_reward, const bool use_rule_reward_for_ego_only)
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
      GOAL_REWARD(goal_reward),
      USE_RULE_REWARD_FOR_EGO_ONLY(use_rule_reward_for_ego_only) {}

bark::models::behavior::RulesMctsStateParameters::RulesMctsStateParameters(
    const commons::ParamsPtr& params)
    : COLLISION_WEIGHT(
          params->GetReal("BehaviorRulesMcts::StateParameters::CollisionWeight",
                          "Penalty for collision", 0)),
      OUT_OF_MAP_WEIGHT(
          params->GetReal("BehaviorRulesMcts::StateParameters::OutOfMapWeight",
                          "Penalty for leaving the map", -800)),
      POTENTIAL_WEIGHT(
          params->GetReal("BehaviorRulesMcts::StateParameters::PotentialWeight",
                          "Weight of the goal potential function", 32.0)),
      ACCELERATION_WEIGHT(params->GetReal(
          "BehaviorRulesMcts::StateParameters::AccelerationWeight",
          "Weight for longitudinal accelerations", 0.0)),
      RADIAL_ACCELERATION_WEIGHT(params->GetReal(
          "BehaviorRulesMcts::StateParameters::RadialAccelerationWeight",
          "Weight for radial acceleration", 0.0)),
      DESIRED_VELOCITY_WEIGHT(params->GetReal(
          "BehaviorRulesMcts::StateParameters::DesiredVelocityWeight",
          "Weight for deviating from the desired velocity", -5.0)),
      LANE_CENTER_WEIGHT(params->GetReal(
          "BehaviorRulesMcts::StateParameters::LaneCenterDeviationWeight",
          "Weight for deviating from the lane center", 0.0)),
      REWARD_VECTOR_SIZE(static_cast<unsigned int>(
          params->GetInt("BehaviorRulesMcts::RewardVectorSize",
                         "Size of the reward vector", 1))),
      PREDICTION_TIME_SPAN(params->GetReal(
          "BehaviorRulesMcts::StateParameters::PredictionTimeSpan",
          "Time between two consecutive states", 0.3)),
      DESIRED_VELOCITY(
          params->GetReal("BehaviorRulesMcts::StateParameters::DesiredVelocity",
                          "Desired driving speed", 10.0)),
      HORIZON(static_cast<unsigned int>(
          params->GetInt("BehaviorRulesMcts::StateParameters::Horizon",
                         "Number of steps until termination", 30))),
      DISCOUNT_FACTOR(params->GetReal("BehaviorRulesMcts::DiscountFactor",
                                      "Discount factor used in MDP problem",
                                      0.9)),
      GOAL_REWARD(params->GetReal(
          "BehaviorRulesMcts::StateParameters::GoalReward",
          "Reward received when reaching the agent's goal.", 0.0)),
      USE_RULE_REWARD_FOR_EGO_ONLY(params->GetBool(
          "BehaviorRulesMcts::StateParameters::UseRuleRewardForEgoOnly",
          "Rewards from Rule evaluations are only used for ego vehicle.",
          false)) {}
