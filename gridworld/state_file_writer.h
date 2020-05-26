// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_STATE_FILE_WRITER_H_
#define GRIDWORLD_STATE_FILE_WRITER_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Core"

class StateFileWriter {
 public:
  StateFileWriter(size_t num_agents, const std::string& filename);
  virtual ~StateFileWriter();
  void write_multi_timestep(const std::vector<Eigen::MatrixXi>& states);

 private:
  int timestamp_;
  std::ofstream ofstream_;
  size_t num_agents_;
  std::string filename_;
};

#endif  // GRIDWORLD_STATE_FILE_WRITER_H_
