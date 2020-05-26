// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gridworld/label_evaluator/evaluator_label_ego_range.h"

EvaluatorLabelEgoRange::EvaluatorLabelEgoRange(const std::string& label_str,
                                               int start, int end)
    : EvaluatorLabelBase(label_str), start_(start), end_(end) {}
std::vector<std::pair<ltl::Label, bool>> EvaluatorLabelEgoRange::evaluate(
    const World& state) const {
  return {{get_label(),
           (state.first.x_pos >= start_ && state.first.x_pos <= end_)}};
}
