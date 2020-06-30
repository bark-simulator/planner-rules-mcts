// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_MULTI_AGENT_H_
#define GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_MULTI_AGENT_H_

#include <string>
#include <utility>
#include <vector>
#include "gridworld/common.hpp"
#include "gridworld/label_evaluator/evaluator_label_base.h"

class EvaluatorLabelMultiAgent : public EvaluatorLabelBase<World> {
 public:
  explicit EvaluatorLabelMultiAgent(const std::string& label_str);
  std::vector<std::pair<Label, bool>> evaluate(
      const World& state) const override;

 protected:
  virtual bool evaluate_agent(const World& state, int agent_id) const = 0;
};

#endif  // GRIDWORLD_LABEL_EVALUATOR_EVALUATOR_LABEL_MULTI_AGENT_H_
