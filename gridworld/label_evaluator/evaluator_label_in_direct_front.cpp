// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gridworld/label_evaluator/evaluator_label_in_direct_front.h"

#include <string>

EvaluatorLabelInDirectFront::EvaluatorLabelInDirectFront(
    const std::string& label_str)
    : EvaluatorLabelMultiAgent(label_str) {}
bool EvaluatorLabelInDirectFront::evaluate_agent(const World& state,
                                                 int agent_id) const {
  if (state.second[agent_id].lane != state.first.lane ||
      state.second[agent_id].x_pos <= state.first.x_pos) {
    // Not on same lane or behind
    return false;
  }
  for (const auto& agent : state.second) {
    if (agent == state.second[agent_id]) {
      continue;
    }
    if (agent.x_pos >= state.first.x_pos &&
        agent.x_pos <= state.second[agent_id].x_pos &&
        agent.lane == state.first.lane) {
      // Found another agent that is closer or equal
      return false;
    }
  }
  return true;
}
