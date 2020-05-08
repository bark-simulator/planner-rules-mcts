// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python_planner_uct.hpp"

namespace py = pybind11;

PYBIND11_MODULE(planner_mvmcts, m) {
  python_planner_uct(m);
}