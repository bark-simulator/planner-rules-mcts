// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gridworld/label_evaluator/evaluator_label_collision.h"

#include <algorithm>

#include "gridworld/label_evaluator/evaluator_label_at_position.h"

bool check_collision(const AgentState &a, const AgentState &b) {
  int o_prev = b.x_pos - b.last_action;
  int upper_o = std::max(o_prev, b.x_pos);
  int lower_o = std::min(o_prev, b.x_pos);

  int e_prev = a.x_pos - a.last_action;
  int upper_e = std::max(e_prev, a.x_pos);
  int lower_e = std::min(e_prev, a.x_pos);

  int overlap = std::min(upper_e, upper_o) - std::max(lower_e, lower_o);
  bool is_overlapping = false;
  if (!((b.last_action != 0 && b.x_pos == e_prev) ||
        (a.last_action != 0 && a.x_pos == o_prev)) ||
      overlap != 0) {
    is_overlapping = overlap >= 0;
  }
  return ((b.lane == a.lane) && (is_overlapping || (a.x_pos == b.x_pos)));
}

EvaluatorLabelCollision::EvaluatorLabelCollision(const std::string &label_str,
                                                 const int merging_point)
    : EvaluatorLabelBase(label_str), merging_point_(merging_point) {}
std::vector<std::pair<Label, bool>> EvaluatorLabelCollision::evaluate(
    const World &state) const {
  // Check if intervals (ego.x_pos - ego.last_action, ego.x_pos] and
  // (other.x_pos - other.last_action, other.x_pos] overlap.
  EvaluatorLabelAtPosition at_merge("at_merge", merging_point_);
  bool ego_at_xing = at_merge.evaluate(state)[0].second;
  World other_world(state.first, std::vector<AgentState>());
  bool result = false;
  for (const auto &agent : state.second) {
    other_world.first = agent;
    bool agent_at_xing = at_merge.evaluate(other_world)[0].second;
    result =
        (ego_at_xing && agent_at_xing) || check_collision(state.first, agent);
    if (result) {
      break;
    }
  }
  return {{get_label(), result}};
}
