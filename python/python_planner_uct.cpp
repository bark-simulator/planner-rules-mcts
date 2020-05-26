// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python/python_planner_uct.hpp"
#include "mcts/random_generator.h"
#include "src/behavior_mcts_multi_agent.hpp"
#include "python/polymorphic_conversion.hpp"
#include "src/mvmcts_state_multi_agent.hpp"

namespace py = pybind11;
using modules::commons::Params;
using modules::models::behavior::BehaviorModel;
using modules::models::behavior::BehaviorUCTMultiAgent;
using modules::models::behavior::BehaviorEGreedyMultiAgent;
using modules::models::behavior::MultiAgentRuleMap;
using modules::models::behavior::MvmctsStateMultiAgent;
using modules::models::behavior::MultiAgentRuleState;
using modules::models::behavior::MvmctsStateParameters;
using modules::models::behavior::MakeMctsParameters;
using modules::world::PredictionSettings;
using modules::world::LabelEvaluators;
using mcts::AgentIdx;
using ltl::RuleMonitor;

void python_planner_uct(py::module m) {

    py::class_ < BehaviorUCTMultiAgent, BehaviorModel,
        std::shared_ptr < BehaviorUCTMultiAgent >> (m, "BehaviorUCTMultiAgent")
            .def(py::init<const modules::commons::ParamsPtr &, const PredictionSettings &,
                          const LabelEvaluators &,
                          const std::vector<std::shared_ptr<RuleMonitor>> &,
                          const MultiAgentRuleMap &>())
            .def("AddAgentRules", &BehaviorUCTMultiAgent::AddAgentRules)
            .def("AddCommonRules", &BehaviorUCTMultiAgent::AddCommonRules)
            .def("AddLabels", &BehaviorUCTMultiAgent::AddLabels)
            .def("GetTree", &BehaviorUCTMultiAgent::GetTree)
            .def("__repr__",
                 [](const BehaviorUCTMultiAgent &m) {
                     return "bark.behavior.BehaviorUCTMultiAgent";
                 })
            .def(py::pickle(
                [](const BehaviorUCTMultiAgent &b) {
                    std::vector<py::tuple> labels;
                    for (const auto &label : b.GetLabelEvaluators()) {
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
                    for (const auto &label_tuple :
                        t[1].cast < std::vector < py::tuple >> ()) {
                        labels.emplace_back(PythonToLabel(label_tuple));
                    }
                    return new BehaviorUCTMultiAgent(
                        PythonToParams(t[0].cast<py::tuple>()),
                        t[4].cast<PredictionSettings>(),
                        labels,
                        t[2].cast < std::vector < std::shared_ptr < ltl::RuleMonitor >> > (),
                        t[3].cast<modules::models::behavior::MultiAgentRuleMap>());
                }));

    py::class_ < BehaviorEGreedyMultiAgent, BehaviorModel,
        std::shared_ptr < BehaviorEGreedyMultiAgent >> (m, "BehaviorEGreedyMultiAgent")
            .def(py::init<const modules::commons::ParamsPtr &, const PredictionSettings &,
                          const LabelEvaluators &,
                          const std::vector<std::shared_ptr<RuleMonitor>> &,
                          const MultiAgentRuleMap &>())
            .def("AddAgentRules", &BehaviorEGreedyMultiAgent::AddAgentRules)
            .def("AddCommonRules", &BehaviorEGreedyMultiAgent::AddCommonRules)
            .def("AddLabels", &BehaviorEGreedyMultiAgent::AddLabels)
            .def("GetTree", &BehaviorEGreedyMultiAgent::GetTree)
            .def("__repr__",
                 [](const BehaviorEGreedyMultiAgent &m) {
                     return "bark.behavior.BehaviorEGreedyMultiAgent";
                 })
            .def(py::pickle(
                [](const BehaviorEGreedyMultiAgent &b) {
                    std::vector<py::tuple> labels;
                    for (const auto &label : b.GetLabelEvaluators()) {
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
                    for (const auto &label_tuple :
                        t[1].cast < std::vector < py::tuple >> ()) {
                        labels.emplace_back(PythonToLabel(label_tuple));
                    }
                    return new BehaviorEGreedyMultiAgent(
                        PythonToParams(t[0].cast<py::tuple>()),
                        t[4].cast<PredictionSettings>(),
                        labels,
                        t[2].cast < std::vector < std::shared_ptr < ltl::RuleMonitor >> > (),
                        t[3].cast<modules::models::behavior::MultiAgentRuleMap>());
                }));

    py::class_ < MvmctsStateMultiAgent, std::shared_ptr < MvmctsStateMultiAgent >> (m, "MvmctsStateMultiAgent")
        .def(py::init<const modules::world::ObservedWorldPtr &,
                      const MultiAgentRuleState &,
                      const MvmctsStateParameters *,
                      const std::vector<AgentIdx> &,
                      unsigned int>())
        .def("Execute", [](const MvmctsStateMultiAgent& state, const mcts::JointAction &joint_action) {
            std::vector<mcts::Reward> rewards;
            auto new_state = state.Execute(joint_action, rewards);
            return std::make_tuple(new_state, rewards);
        })
        .def_property_readonly("observed_world", &MvmctsStateMultiAgent::GetObservedWorld)
        .def("__repr__",
             [](const MvmctsStateMultiAgent &m) {
                 return "bark.behavior.MvmctsStateMultiAgent";
             });

    py::class_ < MvmctsStateParameters, std::shared_ptr < MvmctsStateParameters >> (m, "MvmctsStateParameters")
        .def(py::init<const modules::commons::ParamsPtr &>());
//    py::class_<mcts::MctsParameters, std::shared_ptr<mcts::MctsParameters>>(m, "MctsParameters");

//    m.def("MakeMvmctsStateParameters", &MakeMvmctsStateParameters);
}