// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_UTIL_HPP_
#define MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_UTIL_HPP_

#include <map>
#include "mcts/mcts_parameters.h"
#include "modules/commons/params/params.hpp"
#include "modules/models/behavior/behavior_model.hpp"

namespace modules {
namespace models {
namespace behavior {

mcts::MctsParameters make_mcts_parameters(const commons::ParamsPtr &params);
Eigen::VectorXf StdToEigen(const std::vector<float>& v);

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_UTIL_HPP_
