// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_EGO_RANGE_H_
#define GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_EGO_RANGE_H_

#include <string>
#include <utility>
#include <vector>

#include "gridworld/common.hpp"
#include "gridworld/label_evaluator/evaluator_label_base.h"

class EvaluatorLabelEgoRange : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelEgoRange(const std::string& label_str, int start, int end);
  std::vector<std::pair<Label, bool>> evaluate(
      const World& state) const override;

 private:
  int start_;
  int end_;
};

#endif  // GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_EGO_RANGE_H_
