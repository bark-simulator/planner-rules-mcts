// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_BASE_H_
#define GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_BASE_H_

#include <string>
#include <utility>
#include <vector>
#include "bark/world/evaluation/ltl/label/label.h"

using bark::world::evaluation::Label;

template <class S>
class EvaluatorLabelBase {
 public:
  explicit EvaluatorLabelBase(const std::string &label_str)
      : label_str_(label_str) {}
  virtual std::vector<std::pair<Label, bool>> evaluate(
      const S &state) const = 0;
  Label get_label() const { return Label(label_str_); }
  Label get_agent_label(int agent_id) const {
    return Label(label_str_, agent_id);
  }

 private:
  std::string label_str_;
};

#endif  // GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_BASE_H_
