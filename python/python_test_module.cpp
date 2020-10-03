// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python_planner_mvmcts.hpp"

namespace py = pybind11;

PYBIND11_MODULE(planner_mvmcts, m) {
  python_planner_mvmcts(m);
}