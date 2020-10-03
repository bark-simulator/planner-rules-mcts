// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_DIRECT_FRONT_H_
#define GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_DIRECT_FRONT_H_

#include <string>
#include "gridworld/label_evaluator/evaluator_label_multi_agent.h"

class EvaluatorLabelInDirectFront : public EvaluatorLabelMultiAgent {
 public:
  explicit EvaluatorLabelInDirectFront(const std::string& label_str);

 protected:
  bool evaluate_agent(const World& state, int agent_id) const override;
};

#endif  // GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_DIRECT_FRONT_H_
