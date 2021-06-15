// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python_planner_rules_mcts.hpp"

namespace py = pybind11;

PYBIND11_MODULE(test_planner_rules_mcts, m) {
  python_planner_rules_mcts(m);
}