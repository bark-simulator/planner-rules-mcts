// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gridworld/label_evaluator/evaluator_label_at_position.h"

#include <utility>
#include <vector>

EvaluatorLabelAtPosition::EvaluatorLabelAtPosition(const std::string &label_str,
                                                   const int position)
    : EvaluatorLabelBase(label_str), position_(position) {}
std::vector<std::pair<Label, bool>> EvaluatorLabelAtPosition::evaluate(
    const World &state) const {
  return {{get_label(),
           // Were left, now right of position_
           ((state.first.x_pos - static_cast<int>(state.first.last_action)) <
                position_ &&
            state.first.x_pos >= position_) ||
               // Were right, now left of position
               ((state.first.x_pos -
                 static_cast<int>(state.first.last_action)) > position_ &&
                state.first.x_pos <= position_) ||
               state.first.x_pos == position_}};
}
void EvaluatorLabelAtPosition::set_position(int position) {
  position_ = position;
}
