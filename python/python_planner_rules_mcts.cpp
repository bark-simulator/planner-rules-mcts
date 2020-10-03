// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python/python_planner_rules_mcts.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"
#include "mvmcts/random_generator.h"
#include "src/behavior_rules_mcts.hpp"
#include "src/mvmcts_state.hpp"

namespace py = pybind11;
using bark::commons::Params;
using bark::models::behavior::BehaviorModel;
using bark::models::behavior::BehaviorRulesMctsUct;
using bark::models::behavior::BehaviorRulesMctsGreedy;
using bark::models::behavior::LabelEvaluators;
using bark::models::behavior::LabelEvaluators;
using bark::models::behavior::MvmctsState;
using bark::models::behavior::MultiAgentRuleMap;
using bark::models::behavior::MultiAgentRuleState;
using bark::models::behavior::MultiAgentRuleState;
using bark::models::behavior::MvmctsStateParameters;
using bark::models::behavior::MakeMctsParameters;
using bark::world::PredictionSettings;
using bark::models::behavior::LabelEvaluators;
using ltl::RuleMonitor;
using mvmcts::AgentIdx;

void python_planner_rules_mcts(py::module m) {

    py::class_ <BehaviorRulesMctsUct, BehaviorModel,
        std::shared_ptr <BehaviorRulesMctsUct>> (m, "BehaviorRulesMctsUct")
            .def(py::init<const bark::commons::ParamsPtr &, const PredictionSettings &,
                          const LabelEvaluators &,
                          const std::vector<std::shared_ptr<RuleMonitor>> &,
                          const MultiAgentRuleMap &>())
            .def("AddAgentRules", &BehaviorRulesMctsUct::AddAgentRules)
            .def("AddCommonRules", &BehaviorRulesMctsUct::AddCommonRules)
            .def("AddLabels", &BehaviorRulesMctsUct::AddLabels)
            .def("GetTree", &BehaviorRulesMctsUct::GetTree)
            .def("__repr__",
                 [](const BehaviorRulesMctsUct&m) {
                     return "bark.behavior.BehaviorRulesMctsUct";
                 })
            .def(py::pickle(
                [](const BehaviorRulesMctsUct&b) {
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
                    return new BehaviorRulesMctsUct(
                        PythonToParams(t[0].cast<py::tuple>()),
                        t[4].cast<PredictionSettings>(),
                        labels,
                        t[2].cast < std::vector < std::shared_ptr < ltl::RuleMonitor >> > (),
                        t[3].cast<bark::models::behavior::MultiAgentRuleMap>());
                }));

  py::class_<BehaviorRulesMctsGreedy, BehaviorModel,
             std::shared_ptr<BehaviorRulesMctsGreedy>>(m, "BehaviorRulesMctsGreedy")
      .def(py::init<const bark::commons::ParamsPtr &, const PredictionSettings &,
                          const LabelEvaluators &,
                          const std::vector<std::shared_ptr<RuleMonitor>> &,
                          const MultiAgentRuleMap &>())
        .def("AddAgentRules", &BehaviorRulesMctsGreedy::AddAgentRules)
        .def("AddCommonRules", &BehaviorRulesMctsGreedy::AddCommonRules)
        .def("AddLabels", &BehaviorRulesMctsGreedy::AddLabels)
        .def("GetTree", &BehaviorRulesMctsGreedy::GetTree)
        .def("__repr__",
             [](const BehaviorRulesMctsGreedy& m) {
               return "bark.behavior.BehaviorRulesMctsGreedy";
             })
            .def(py::pickle(
            [](const BehaviorRulesMctsGreedy& b) {
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
                    return new BehaviorRulesMctsGreedy(
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
      .def("Execute",
           [](const MvmctsState& state,
              const mvmcts::JointAction& joint_action) {
             std::vector<mvmcts::Reward> rewards;
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