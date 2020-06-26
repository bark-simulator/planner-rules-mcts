// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef SRC_BEHAVIOR_MCTS_MULTI_AGENT_HPP_
#define SRC_BEHAVIOR_MCTS_MULTI_AGENT_HPP_

#include <cxxabi.h>
#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/prediction/prediction_settings.hpp"
#include "glog/logging.h"
#include "ltl/rule_state.h"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/mcts_parameters.h"
#include "mcts/statistics/e_greedy_statistic.h"
#include "mcts/statistics/thres_uct_statistic.h"
#include "src/mvmcts_state.hpp"
#include "src/util.hpp"

namespace bark {
namespace world {
class ObservedWorld;
}
namespace models {
namespace behavior {

using bark::models::dynamic::SingleTrackModel;
using bark::world::AgentMap;
using bark::world::ObservedWorldPtr;
using bark::world::prediction::PredictionSettings;
using commons::ParamsPtr;
using ltl::RuleMonitor;
using ltl::RuleState;
using mcts::MctsParameters;
using mcts::ObjectiveVec;
typedef std::unordered_map<AgentId, std::vector<std::shared_ptr<RuleMonitor>>>
    MultiAgentRuleMap;
typedef std::pair<mcts::Reward::Scalar, geometry::Line> ValueLinePair;
typedef std::vector<ValueLinePair> ValueLinePairVector;

template <class Stat>
class BehaviorMvmcts : public bark::models::behavior::BehaviorModel {
 public:
  using MctsNode = typename mcts::Mcts<MvmctsStateMultiAgent, Stat, Stat,
                                       mcts::RandomHeuristic>::StageNodeSPtr;

  BehaviorMvmcts(
      const ParamsPtr& params, const PredictionSettings& prediction_settings,
      const LabelEvaluators& label_evaluators,
      const std::vector<std::shared_ptr<RuleMonitor>>& common_rules,
      const MultiAgentRuleMap& agent_rules);

  ~BehaviorMvmcts() = default;

  Trajectory Plan(float delta_time,
                  const world::ObservedWorld& observed_world) override;

  std::shared_ptr<BehaviorModel> Clone() const override;

  void AddAgentRules(const std::vector<AgentId>& agent_ids,
                     const std::vector<std::shared_ptr<RuleMonitor>>& rules);
  void AddCommonRules(const std::vector<std::shared_ptr<RuleMonitor>>& rules);

  void AddLabels(const LabelEvaluators& label_evaluators);

  ValueLinePairVector GetTree(size_t value_idx = 0);
  const std::vector<std::shared_ptr<RuleMonitor>>& GetCommonRules() const;
  const MultiAgentRuleMap& GetAgentRules() const;
  const LabelEvaluators& GetLabelEvaluators() const;
  const MctsParameters& GetMctsParameters() const;
  const PredictionSettings& GetPredictionSettings() const;

  void SetMctsParameters(const MctsParameters& mcts_parameters);

 private:
  std::vector<int> GetNewAgents(const AgentMap& agent_map);
  std::vector<AgentIdx> GetAgentIdMap(
      const world::ObservedWorld& observed_world) const;
  void MakeRuleStates(const std::vector<int>& new_agent_ids);
  void AddKnownAgents(const std::vector<int>& agent_ids);
  ValueLinePairVector DfsTree(BehaviorMvmcts::MctsNode root,
                              const ValueLinePair& prefix, size_t value_idx);

  std::vector<std::shared_ptr<RuleMonitor>> common_rules_;
  MultiAgentRuleMap agent_rules_;
  MultiAgentRuleState multi_agent_rule_state_;
  bark::world::prediction::PredictionSettings prediction_settings_;
  unsigned int max_num_iterations_;
  unsigned int max_search_time_;
  unsigned int random_seed_;
  bool dump_tree_;
  LabelEvaluators label_evaluators_;
  size_t reward_vec_size_;
  MctsParameters mcts_parameters_;
  MvmctsStateParameters state_params_;
  std::set<AgentId> known_agents_;
  MctsNode root_;
  const bool multi_agent_;
};

typedef BehaviorMvmcts<mcts::ThresUCTStatistic> BehaviorMvmctsUct;
typedef BehaviorMvmcts<mcts::EGreedyStatistic> BehaviorMvmctsEGreedy;

template <class Stat>
BehaviorMvmcts<Stat>::BehaviorMvmcts(
    const ParamsPtr& params, const PredictionSettings& prediction_settings,
    const LabelEvaluators& label_evaluators,
    const std::vector<std::shared_ptr<RuleMonitor>>& common_rules,
    const MultiAgentRuleMap& agent_rules)
    : BehaviorModel(params),
      common_rules_(common_rules),
      agent_rules_(agent_rules),
      prediction_settings_(prediction_settings),
      max_num_iterations_(
          params->GetInt("BehaviorMCTSAgent::MaxNumIterations",
                         "Maximum number of mcts search iterations", 2000)),
      max_search_time_(params->GetInt("BehaviorMCTSAgent::MaxSearchTime",
                                      "Maximum search time in milliseconds",
                                      std::numeric_limits<int>::max())),
      random_seed_(params->GetInt(
          "BehaviorMCTSAgent::RandomSeed",
          "Random seed applied used during search process", 1000)),
      dump_tree_(params->GetBool(
          "BehaviorMCTSAgent::DumpTree",
          "If true, tree is dumped to dot file after planning", false)),
      label_evaluators_(label_evaluators),
      reward_vec_size_(
          params->GetInt("BehaviorMCTSAgent::RewardVectorSize",
                         "Number of dimensions of the reward vector", 1)),
      mcts_parameters_(bark::models::behavior::MakeMctsParameters(params)),
      state_params_(params),
      multi_agent_(params->GetBool("BehaviorMCTSAgent::MultiAgent",
                                   "True for multi-agent planning", true)) {
  mcts::RandomGenerator::random_generator_ = std::mt19937(random_seed_);
}

template <class Stat>
dynamic::Trajectory BehaviorMvmcts<Stat>::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  ObservedWorldPtr mcts_observed_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
  mcts_observed_world->SetupPrediction(prediction_settings_);

  // SETUP MCTS
  mcts::Mcts<MvmctsStateMultiAgent, Stat, Stat, mcts::RandomHeuristic> mcts(
      mcts_parameters_);

  // SETUP MCTS STATE
  auto ego_model = std::dynamic_pointer_cast<BehaviorMPMacroActions>(
      prediction_settings_.ego_prediction_model_);
  auto new_agents = GetNewAgents(observed_world.GetAgents());
  MakeRuleStates(new_agents);
  AddKnownAgents(new_agents);
  auto agent_ids = GetAgentIdMap(observed_world);
  MvmctsStateMultiAgent mcts_state(mcts_observed_world, multi_agent_rule_state_,
                                   &state_params_, agent_ids,
                                   state_params_.HORIZON, &label_evaluators_);

  // Transit automata from last planning step to current
  mcts_state.EvaluateRules();

  multi_agent_rule_state_ = mcts_state.GetMultiAgentRuleState();
  for (const auto& rs :
       multi_agent_rule_state_[observed_world.GetEgoAgentId()]) {
    VLOG(2) << rs;
  }
  auto available_mp_idx = ego_model->GetValidPrimitives(mcts_observed_world);
  auto primitives = ego_model->GetMotionPrimitives();
  std::stringstream outs;
  outs << "Available Primitives: ";
  int status;
  for (const auto& prim : available_mp_idx) {
    outs << abi::__cxa_demangle(typeid(*(primitives.at(prim))).name(), 0, 0,
                                &status)
         << ", ";
  }
  VLOG(1) << outs.str();

  // MCTS SEARCH
  mcts.Search(mcts_state, max_search_time_, max_num_iterations_);

  mcts::ActionIdx best_action = mcts.ReturnBestAction()[0];
  SetLastAction(DiscreteAction(best_action));

  if (dump_tree_) {
    std::stringstream filename;
    filename << "/tmp/tree_dot_file_" << delta_time;
    mcts.PrintTreeToDotFile(filename.str());
  }

  ego_model->ActionToBehavior(BehaviorMotionPrimitives::MotionIdx(best_action));
  auto traj = ego_model->Plan(delta_time, observed_world);

  // Save root node of mcts for tree visualization
  root_ = mcts.GetRoot();

  LOG(INFO) << "BehaviorMvmcts, iterations: " << mcts.NumIterations()
            << ", best action: " << best_action
            << ", current state: " << traj.row(traj.rows() - 1);

  SetLastTrajectory(traj);
  return traj;
}

/// Rules to be added to each new agent
/// \tparam Stat
/// \param rules
template <class Stat>
void BehaviorMvmcts<Stat>::AddCommonRules(
    const std::vector<std::shared_ptr<RuleMonitor>>& rules) {
  common_rules_.insert(common_rules_.end(), rules.begin(), rules.end());
  // Print automata to dot file
  //  int i = 0;
  //  char obuf[50];
  //  for(const auto &r : rules) {
  //    std::sprintf(obuf, "/tmp/aut_%d.dot", i);
  //    r->PrintToDot(obuf);
  //    ++i;
  //  }
}

/// Rules to be added only to agents with specified id
/// \tparam Stat
/// \param agent_ids AgentIDs of agent to add rules to
/// \param rules
template <class Stat>
void BehaviorMvmcts<Stat>::AddAgentRules(
    const std::vector<AgentId>& agent_ids,
    const std::vector<std::shared_ptr<RuleMonitor>>& rules) {
  // Need to store rules separately that if there appears a new agent, we can
  // create new rule states
  for (const auto& agent_id : agent_ids) {
    auto it = agent_rules_.find(agent_id);
    if (it == agent_rules_.end()) {
      agent_rules_.insert(
          {agent_id, std::vector<std::shared_ptr<RuleMonitor>>()});
    }
    agent_rules_.at(agent_id).insert(agent_rules_.at(agent_id).end(),
                                     rules.begin(), rules.end());
  }
}
template <class Stat>
void BehaviorMvmcts<Stat>::MakeRuleStates(
    const std::vector<int>& new_agent_ids) {
  std::vector<int> current_agent_ids;
  std::set_union(new_agent_ids.begin(), new_agent_ids.end(),
                 known_agents_.begin(), known_agents_.end(),
                 std::back_inserter(current_agent_ids));

  for (const auto& agent_id : new_agent_ids) {
    auto it =
        multi_agent_rule_state_.insert({agent_id, std::vector<RuleState>()})
            .first;
    std::vector<int> others = current_agent_ids;
    others.erase(std::find(others.begin(), others.end(), agent_id));
    // Default rules
    for (const auto& rule : common_rules_) {
      // All agents are new
      auto rs = rule->MakeRuleState(others, {});
      it->second.insert(it->second.end(), rs.begin(), rs.end());
    }
    // Rules that have been specified for a specific agent only
    auto agent_rule_it = agent_rules_.find(agent_id);
    if (agent_rule_it != agent_rules_.end()) {
      for (const auto& rule : agent_rule_it->second) {
        // All agents are new
        auto rs = rule->MakeRuleState(others, {});
        it->second.insert(it->second.end(), rs.begin(), rs.end());
      }
    }
  }

  // Add missing permutations to existing agents
  for (const auto& agent_id : known_agents_) {
    auto it = multi_agent_rule_state_.find(agent_id);
    std::set<AgentId> others = known_agents_;
    others.erase(agent_id);
    std::vector<int> others_vec;
    std::copy(others.begin(), others.end(), std::back_inserter(others_vec));
    // Default rules
    for (const auto& rule : common_rules_) {
      if (rule->IsAgentSpecific()) {
        auto rs = rule->MakeRuleState(new_agent_ids, others_vec);
        it->second.insert(it->second.end(), rs.begin(), rs.end());
      }
    }
    // Rules that have been specified for a specific agent only
    auto agent_rule_it = agent_rules_.find(agent_id);
    if (agent_rule_it != agent_rules_.end()) {
      for (const auto& rule : agent_rule_it->second) {
        if (rule->IsAgentSpecific()) {
          auto rs = rule->MakeRuleState(new_agent_ids, others_vec);
          it->second.insert(it->second.end(), rs.begin(), rs.end());
        }
      }
    }
  }
}
template <class Stat>
const std::vector<std::shared_ptr<RuleMonitor>>&
BehaviorMvmcts<Stat>::GetCommonRules() const {
  return common_rules_;
}
template <class Stat>
const MultiAgentRuleMap& BehaviorMvmcts<Stat>::GetAgentRules() const {
  return agent_rules_;
}
template <class Stat>
void BehaviorMvmcts<Stat>::AddLabels(
    const LabelEvaluators& label_evaluators) {
  label_evaluators_.insert(label_evaluators_.end(), label_evaluators.begin(),
                           label_evaluators.end());
}
template <class Stat>
const MctsParameters& BehaviorMvmcts<Stat>::GetMctsParameters() const {
  return mcts_parameters_;
}
template <class Stat>
void BehaviorMvmcts<Stat>::SetMctsParameters(
    const MctsParameters& mcts_parameters) {
  mcts_parameters_ = mcts_parameters;
}
template <class Stat>
std::vector<int> BehaviorMvmcts<Stat>::GetNewAgents(
    const AgentMap& agent_map) {
  std::vector<int> sorted_agent_ids;
  // Convert type to int
  std::transform(
      agent_map.begin(), agent_map.end(), std::back_inserter(sorted_agent_ids),
      [](const AgentMap::value_type& e) { return static_cast<int>(e.first); });
  // Need sorted container for set operations
  std::sort(sorted_agent_ids.begin(), sorted_agent_ids.end());
  std::vector<int> new_agents;
  // new_agents = agent_ids \ known_agents
  std::set_difference(sorted_agent_ids.begin(), sorted_agent_ids.end(),
                      known_agents_.begin(), known_agents_.end(),
                      std::back_inserter(new_agents));
  return new_agents;
}
//! Create a map from BARKs agent ids to a range of consecutive natural numbers
//! \warning The map does not include non-interacting agents!
//! \param observed_world
//! \return A vector of interacting agent ids. Ego is always stored in the first
//! entry
template <class Stat>
std::vector<AgentIdx> BehaviorMvmcts<Stat>::GetAgentIdMap(
    const world::ObservedWorld& observed_world) const {
  std::vector<mcts::AgentIdx> agent_ids;
  if (multi_agent_) {
    world::AgentMap agent_map = observed_world.GetOtherAgents();
    agent_ids.emplace_back(observed_world.GetEgoAgent()->GetAgentId());
    for (const auto& agent : agent_map) {
      if (prediction_settings_.specific_prediction_agents_.count(agent.first) ==
          0) {
        agent_ids.emplace_back(agent.first);
      }
    }
  } else {
    agent_ids.emplace_back(observed_world.GetEgoAgentId());
  }
  return agent_ids;
}
template <class Stat>
void BehaviorMvmcts<Stat>::AddKnownAgents(
    const std::vector<int>& agent_ids) {
  known_agents_.insert(agent_ids.begin(), agent_ids.end());
}
template <class Stat>
const LabelEvaluators& BehaviorMvmcts<Stat>::GetLabelEvaluators()
    const {
  return label_evaluators_;
}
template <class Stat>
const PredictionSettings& BehaviorMvmcts<Stat>::GetPredictionSettings()
    const {
  return prediction_settings_;
}

template <class Stat>
ValueLinePairVector BehaviorMvmcts<Stat>::GetTree(size_t value_idx) {
  ValueLinePairVector lines;
  if (root_) {
    lines = DfsTree(root_, ValueLinePair(0.0f, geometry::Line()), value_idx);
  }
  return lines;
}
template <class Stat>
ValueLinePairVector BehaviorMvmcts<Stat>::DfsTree(BehaviorMvmcts::MctsNode root, const ValueLinePair& prefix,
    size_t value_idx) {
  ValueLinePair new_prefix(prefix);
  auto ego_agent = root->GetState()->GetObservedWorld()->GetEgoAgent();
  if (ego_agent) {
    new_prefix.second.AddPoint(ego_agent->GetCurrentPosition());
  }
  ValueLinePairVector lines;
  auto child_map = root->GetChildren();
  if (child_map.size() > 0) {
    for (const auto& child : child_map) {
      new_prefix.first +=
          root->GetJointRewards().find(child.first)->second.at(0)(value_idx);
      auto res = DfsTree(child.second, new_prefix, value_idx);
      lines.insert(lines.end(), res.begin(), res.end());
    }
  } else {
    new_prefix.first += root->GetValue()[0](value_idx);
    lines.push_back(new_prefix);
  }
  return lines;
}
template <class Stat>
std::shared_ptr<BehaviorModel> BehaviorMvmcts<Stat>::Clone() const {
  return std::dynamic_pointer_cast<BehaviorModel>(
      std::make_shared<BehaviorMvmcts<Stat>>(*this));
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // SRC_BEHAVIOR_MCTS_MULTI_AGENT_HPP_
