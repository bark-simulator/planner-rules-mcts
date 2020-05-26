// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gridworld/label_evaluator/evaluator_label_multi_agent.h"

#include <utility>
#include <vector>

EvaluatorLabelMultiAgent::EvaluatorLabelMultiAgent(const std::string& label_str)
    : EvaluatorLabelBase(label_str) {}
std::vector<std::pair<ltl::Label, bool>> EvaluatorLabelMultiAgent::evaluate(
    const World& state) const {
  std::vector<std::pair<ltl::Label, bool>> agent_labels;
  for (size_t i = 0; i < state.second.size(); ++i) {
    agent_labels.emplace_back(std::make_pair(
        get_agent_label(state.second[i].id), evaluate_agent(state, i)));
  }
  return agent_labels;
}
