// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_UTIL_HPP_
#define MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_UTIL_HPP_

#include <map>
#include "bark/commons/params/params.hpp"
#include "bark/models/behavior/behavior_model.hpp"
#include "mvmcts/mvmcts_parameters.h"

namespace bark {
namespace models {
namespace behavior {

mvmcts::MvmctsParameters MakeMctsParameters(const commons::ParamsPtr& params);
Eigen::VectorXf StdToEigen(const std::vector<float>& v);

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_UTIL_HPP_
