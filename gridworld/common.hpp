// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_COMMON_HPP_
#define GRIDWORLD_COMMON_HPP_

#include <map>
#include <utility>
#include <vector>

#include "mcts/state.h"

using mcts::ActionIdx;

enum class Actions { WAIT = 0, FORWARD = 1, BACKWARD = -1, NUM = 4 };

enum Rule {
  NO_COLLISION = 0,
  ZIP,
  NUM,
};

const std::map<ActionIdx, Actions> idx_to_action = {
    {0, Actions::FORWARD},
    {1, Actions::WAIT},
    {2, Actions::BACKWARD},
};

const std::map<Actions, ActionIdx> action_to_idx = {
    {Actions::FORWARD, 0},
    {Actions::WAIT, 1},
    {Actions::BACKWARD, 2},
};

static inline Actions aconv(const ActionIdx &action) {
  return idx_to_action.at(action);
}

static inline ActionIdx aconv(const Actions &action) {
  return action_to_idx.at(action);
}

typedef struct AgentState {
  AgentState()
      : id(id_counter++),
        x_pos(0),
        last_action(static_cast<int>(Actions::FORWARD)),
        lane(++lane_counter),
        init_lane(lane) {}
  AgentState(const int &id, const int &x, const int &last_action, int lane,
             int init_lane)
      : id(id),
        x_pos(x),
        last_action(last_action),
        lane(lane),
        init_lane(init_lane) {}
  friend bool operator==(const AgentState &lhs, const AgentState &rhs);
  friend bool operator!=(const AgentState &lhs, const AgentState &rhs);
  int id;
  int x_pos;
  int last_action;
  int lane;
  int init_lane;
  static int lane_counter;
  static int id_counter;
} AgentState;

typedef std::pair<AgentState, std::vector<AgentState>> World;

template <typename T>
inline std::ostream& operator<<(std::ostream& os, std::vector<T> const& d) {
  os << "[";
  for (typename std::vector<T>::const_iterator ii = d.begin(); ii != d.end();
       ++ii) {
    os << *ii;
    if (ii >= d.end() - 1) {
      break;
    }
    os << ", ";
  }
  os << "]";
  return os;
}

#endif  // GRIDWORLD_COMMON_HPP_
