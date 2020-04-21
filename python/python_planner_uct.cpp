// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python/python_planner_uct.hpp"
#include "mcts/random_generator.h"
#include "src/behavior_mcts_multi_agent.hpp"
#include "python/polymorphic_conversion.hpp"

namespace py = pybind11;
using modules::commons::Params;
using modules::models::behavior::BehaviorModel;
using modules::models::behavior::BehaviorUCTMultiAgent;
using modules::models::behavior::BehaviorEGreedyMultiAgent;
using modules::world::PredictionSettings;
using modules::world::LabelEvaluators;
using ltl::RuleMonitor;
using modules::models::behavior::MultiAgentRuleMap;

void python_planner_uct(py::module m) {

  py::class_<BehaviorUCTMultiAgent, BehaviorModel,
             std::shared_ptr<BehaviorUCTMultiAgent>>(m, "BehaviorUCTMultiAgent")
      .def(py::init<const modules::commons::ParamsPtr&, const PredictionSettings &,
                    const LabelEvaluators &,
                    const std::vector<std::shared_ptr<RuleMonitor>> &,
                    const MultiAgentRuleMap &>())
      .def("add_agent_rules", &BehaviorUCTMultiAgent::add_agent_rules)
      .def("add_common_rules", &BehaviorUCTMultiAgent::add_common_rules)
      .def("AddLabels", &BehaviorUCTMultiAgent::add_labels)
      .def("get_tree", &BehaviorUCTMultiAgent::get_tree)
      .def("__repr__",
           [](const BehaviorUCTMultiAgent& m) {
             return "bark.behavior.BehaviorUCTMultiAgent";
           })
      .def(py::pickle(
          [](const BehaviorUCTMultiAgent& b) {
            std::vector<py::tuple> labels;
            for (const auto& label : b.GetLabelEvaluators()) {
              labels.emplace_back(LabelToPython(label));
            }
            // TODO: This is enough for a model that has not been used yet.
            // Otherwise would also need root, known_agents, rule_states
            return py::make_tuple(ParamsToPython(b.GetParams()), labels,
                                  b.GetCommonRules(), b.GetAgentRules(), b.GetPredictionSettings());
          },
          [](py::tuple t) {
            if (t.size() != 5)
              throw std::runtime_error("Invalid behavior model state!");
            std::vector<LabelEvaluatorPtr> labels;
            for (const auto& label_tuple :
                 t[1].cast<std::vector<py::tuple>>()) {
              labels.emplace_back(PythonToLabel(label_tuple));
            }
            return new BehaviorUCTMultiAgent(
                PythonToParams(t[0].cast<py::tuple>()),
                t[4].cast<PredictionSettings>(),
                labels,
                t[2].cast<std::vector<std::shared_ptr<ltl::RuleMonitor>>>(),
                t[3].cast<modules::models::behavior::MultiAgentRuleMap>());
          }));

  py::class_<BehaviorEGreedyMultiAgent, BehaviorModel,
             std::shared_ptr<BehaviorEGreedyMultiAgent>>(m, "BehaviorEGreedyMultiAgent")
      .def(py::init<const modules::commons::ParamsPtr&, const PredictionSettings &,
                    const LabelEvaluators &,
                    const std::vector<std::shared_ptr<RuleMonitor>> &,
                    const MultiAgentRuleMap &>())
      .def("add_agent_rules", &BehaviorEGreedyMultiAgent::add_agent_rules)
      .def("add_common_rules", &BehaviorEGreedyMultiAgent::add_common_rules)
      .def("AddLabels", &BehaviorEGreedyMultiAgent::add_labels)
      .def("get_tree", &BehaviorEGreedyMultiAgent::get_tree)
      .def("__repr__",
           [](const BehaviorEGreedyMultiAgent& m) {
             return "bark.behavior.BehaviorEGreedyMultiAgent";
           })
      .def(py::pickle(
          [](const BehaviorEGreedyMultiAgent& b) {
            std::vector<py::tuple> labels;
            for (const auto& label : b.GetLabelEvaluators()) {
              labels.emplace_back(LabelToPython(label));
            }
            // TODO: This is enough for a model that has not been used yet.
            // Otherwise would also need root, known_agents, rule_states
            return py::make_tuple(ParamsToPython(b.GetParams()), labels,
                                  b.GetCommonRules(), b.GetAgentRules(), b.GetPredictionSettings());
          },
          [](py::tuple t) {
            if (t.size() != 5)
              throw std::runtime_error("Invalid behavior model state!");
            std::vector<LabelEvaluatorPtr> labels;
            for (const auto& label_tuple :
                t[1].cast<std::vector<py::tuple>>()) {
              labels.emplace_back(PythonToLabel(label_tuple));
            }
            return new BehaviorEGreedyMultiAgent(
                PythonToParams(t[0].cast<py::tuple>()),
                t[4].cast<PredictionSettings>(),
                labels,
                t[2].cast<std::vector<std::shared_ptr<ltl::RuleMonitor>>>(),
                t[3].cast<modules::models::behavior::MultiAgentRuleMap>());
          }));
}