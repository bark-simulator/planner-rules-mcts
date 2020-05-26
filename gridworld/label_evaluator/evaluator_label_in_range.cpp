// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gridworld/label_evaluator/evaluator_label_in_range.h"

#include <string>

EvaluatorLabelInRange::EvaluatorLabelInRange(const std::string& label_str,
                                             int range_start, int range_end)
    : EvaluatorLabelMultiAgent(label_str),
      range_start_(range_start),
      range_end_(range_end) {}
bool EvaluatorLabelInRange::evaluate_agent(const World& state,
                                           int agent_id) const {
  return state.second[agent_id].x_pos >= range_start_ &&
         state.second[agent_id].x_pos <= range_end_;
}
