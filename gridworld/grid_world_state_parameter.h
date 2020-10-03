// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_GRID_WORLD_STATE_PARAMETER_H_
#define GRIDWORLD_GRID_WORLD_STATE_PARAMETER_H_

#include <vector>

#include "ltl/rule_monitor.h"

struct GridWorldStateParameter {
  unsigned int num_other_agents;
  int state_x_length;
  int ego_goal_reached_position;
  int terminal_depth_;
  int merging_point;

  float speed_deviation_weight;
  float acceleration_weight;
  float potential_weight;

  size_t reward_vec_size;

  std::vector<int> action_map;
};

#endif  // GRIDWORLD_GRID_WORLD_STATE_PARAMETER_H_
