// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_TEST_RUNNER_H_
#define GRIDWORLD_TEST_RUNNER_H_

#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include "external/com_github_google_glog/_virtual_includes/default_glog_headers/glog/logging.h"

#include "gridworld/grid_world_env.h"

using Eigen::ArrayXi;
using Eigen::MatrixXf;
using mcts::JointReward;
using std::ofstream;
using std::ostream;
using std::stringstream;
using std::vector;

class TestRunner {
 public:
  struct Result {
    Result() : collision(false), violation(false) {}
    friend ostream &operator<<(ostream &os, const Result &result);
    static ostream &WriteHeader(ostream &os);
    int pos;
    float value;
    bool collision;
    bool violation;

   private:
    static inline const char *BoolToString(bool b) {
      return b ? "true" : "false";
    }
  };

  explicit TestRunner(std::shared_ptr<BaseTestEnv> test_env)
      : latest_test_env_(std::move(test_env)), q_val_fname_("/tmp/q_val.dat") {}
  Result RunTest(size_t num_iter, int max_steps = 40);
  const std::shared_ptr<BaseTestEnv> &GetLatestTestEnv() const;
  Eigen::VectorXi GetStateVector() const;
  void SetQValFname(const std::string &q_val_fname);

 private:
  void PrintLabels();
  void PrintRuleStates();
  std::shared_ptr<BaseTestEnv> latest_test_env_;
  std::string q_val_fname_;
};

#endif  // GRIDWORLD_TEST_RUNNER_H_