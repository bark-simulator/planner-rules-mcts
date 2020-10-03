// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_RANGE_H_
#define GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_RANGE_H_

#include <string>

#include "gridworld/label_evaluator/evaluator_label_multi_agent.h"

class EvaluatorLabelInRange : public EvaluatorLabelMultiAgent {
 public:
  EvaluatorLabelInRange(const std::string& label_str, int range_start,
                        int range_end);

 protected:
  bool evaluate_agent(const World& state, int agent_id) const override;

 private:
  int range_start_;
  int range_end_;
};

#endif  // GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_RANGE_H_
