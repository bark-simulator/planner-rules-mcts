// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python/python_planner_mvmcts.hpp"
#include "mcts/random_generator.h"
#include "src/behavior_mvmcts.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"
#include "src/mvmcts_state.hpp"

namespace py = pybind11;
using bark::commons::Params;
using bark::models::behavior::BehaviorModel;
using bark::models::behavior::BehaviorModel;
using bark::models::behavior::LabelEvaluators;
using bark::models::behavior::MultiAgentRuleMap;
using bark::models::behavior::MultiAgentRuleState;
using bark::models::behavior::MultiAgentRuleState;
using bark::models::behavior::MvmctsStateParameters;
using bark::models::behavior::MakeMctsParameters;
using bark::world::PredictionSettings;
using bark::models::behavior::LabelEvaluators;
using mcts::AgentIdx;
using ltl::RuleMonitor;

void python_planner_mvmcts(py::module m) {

    py::class_ <BehaviorMvmctsUct, BehaviorModel,
        std::shared_ptr <BehaviorMvmctsUct>> (m, "BehaviorMvmctsUct")
            .def(py::init<const bark::commons::ParamsPtr &, const PredictionSettings &,
                          const LabelEvaluators &,
                          const std::vector<std::shared_ptr<RuleMonitor>> &,
                          const MultiAgentRuleMap &>())
            .def("AddAgentRules", &BehaviorMvmctsUct::AddAgentRules)
            .def("AddCommonRules", &BehaviorMvmctsUct::AddCommonRules)
            .def("AddLabels", &BehaviorMvmctsUct::AddLabels)
            .def("GetTree", &BehaviorMvmctsUct::GetTree)
            .def("__repr__",
                 [](const BehaviorMvmctsUct&m) {
                     return "bark.behavior.BehaviorMvmctsUct";
                 })
            .def(py::pickle(
                [](const BehaviorMvmctsUct&b) {
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
                    LabelEvaluators labels;
                    for (const auto &label_tuple :
                        t[1].cast < std::vector < py::tuple >> ()) {
                        labels.emplace_back(PythonToLabel(label_tuple));
                    }
                    return new BehaviorMvmctsUct(
                        PythonToParams(t[0].cast<py::tuple>()),
                        t[4].cast<PredictionSettings>(),
                        labels,
                        t[2].cast < std::vector < std::shared_ptr < ltl::RuleMonitor >> > (),
                        t[3].cast<bark::models::behavior::MultiAgentRuleMap>());
                }));

    py::class_ <BehaviorMvmctsEGreedy, BehaviorModel,
        std::shared_ptr <BehaviorMvmctsEGreedy>> (m, "BehaviorMvmctsEGreedy")
            .def(py::init<const bark::commons::ParamsPtr &, const PredictionSettings &,
                          const LabelEvaluators &,
                          const std::vector<std::shared_ptr<RuleMonitor>> &,
                          const MultiAgentRuleMap &>())
            .def("AddAgentRules", &BehaviorMvmctsEGreedy::AddAgentRules)
            .def("AddCommonRules", &BehaviorMvmctsEGreedy::AddCommonRules)
            .def("AddLabels", &BehaviorMvmctsEGreedy::AddLabels)
            .def("GetTree", &BehaviorMvmctsEGreedy::GetTree)
            .def("__repr__",
                 [](const BehaviorMvmctsEGreedy&m) {
                     return "bark.behavior.BehaviorMvmctsEGreedy";
                 })
            .def(py::pickle(
                [](const BehaviorMvmctsEGreedy&b) {
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
                    LabelEvaluators labels;
                    for (const auto &label_tuple :
                        t[1].cast < std::vector < py::tuple >> ()) {
                        labels.emplace_back(PythonToLabel(label_tuple));
                    }
                    return new BehaviorMvmctsEGreedy(
                        PythonToParams(t[0].cast<py::tuple>()),
                        t[4].cast<PredictionSettings>(),
                        labels,
                        t[2].cast < std::vector < std::shared_ptr < ltl::RuleMonitor >> > (),
                        t[3].cast<bark::models::behavior::MultiAgentRuleMap>());
                }));

    py::class_ <MvmctsState, std::shared_ptr <MvmctsState>> (m, "MvmctsState")
        .def(py::init<const bark::world::ObservedWorldPtr &,
                      const MultiAgentRuleState &,
                      const MvmctsStateParameters *,
                      const std::vector<AgentIdx> &,
                      unsigned int,
                      const LabelEvaluators*>())
        .def("Execute", [](const MvmctsState& state, const mcts::JointAction &joint_action) {
            std::vector<mcts::Reward> rewards;
            auto new_state = state.Execute(joint_action, rewards);
            return std::make_tuple(new_state, rewards);
        })
        .def_property_readonly("observed_world", &MvmctsState::GetObservedWorld)
        .def("__repr__",
             [](const MvmctsState&m) {
                 return "bark.behavior.MvmctsState";
             });

    py::class_ < MvmctsStateParameters, std::shared_ptr < MvmctsStateParameters >> (m, "MvmctsStateParameters")
        .def(py::init<const bark::commons::ParamsPtr &>());

}