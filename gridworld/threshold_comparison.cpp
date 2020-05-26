// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <vector>

#include "gridworld/state_file_writer.h"
#include "gridworld/test_runner.h"
#include "mcts/statistics/e_greedy_statistic.h"
#include "mcts/statistics/thres_uct_statistic.h"

void run(const ObjectiveVec& thres, int test_no) {
  char fname[100];
  sprintf(fname, "/tmp/threshold_compare_%d.dat", test_no);
  StateFileWriter sfw(3, std::string(fname));
  auto test_runner = std::make_unique<TestRunner>(
      std::make_shared<CrossingTestEnv<ThresUCTStatistic>>(thres));
  sprintf(fname, "/tmp/threshold_compare_q_val_%d.dat", test_no);
  test_runner->SetQValFname(std::string(fname));
  auto res = test_runner->RunTest(5000, 16);
  TestRunner::Result::WriteHeader(LOG(INFO));
  LOG(INFO) << res;
  sfw.write_multi_timestep(test_runner->GetLatestTestEnv()->state_history);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_v = 1;
  FLAGS_alsologtostderr = true;
  mcts::RandomGenerator::random_generator_ = std::mt19937(1000);

  std::vector<ObjectiveVec> thresholds(3, ObjectiveVec::Zero(3));
  thresholds[0] << -1.0, -1.0, std::numeric_limits<ObjectiveVec::Scalar>::max();
  thresholds[1] << -0.37, -0.37,
      std::numeric_limits<ObjectiveVec::Scalar>::max();
  thresholds[2] << -0.1, -0.8, std::numeric_limits<ObjectiveVec::Scalar>::max();

  for (size_t i = 0; i < thresholds.size(); ++i) {
    run(thresholds[i], i);
  }

  return 0;
}
