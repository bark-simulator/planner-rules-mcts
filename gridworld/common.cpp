// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "common.hpp"

int AgentState::lane_counter = 0;
int AgentState::id_counter = 0;
bool operator==(const AgentState& lhs, const AgentState& rhs) {
  return lhs.id == rhs.id;
}
bool operator!=(const AgentState& lhs, const AgentState& rhs) {
  return !(rhs == lhs);
}
