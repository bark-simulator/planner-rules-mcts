// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <vector>
#include "gtest/gtest.h"

#include "gridworld/state_file_writer.h"
#include "gridworld/test_runner.h"
#include "mvmcts/statistics/thres_greedy_statistic.h"
#include "mvmcts/statistics/thres_uct_statistic.h"

#define MERGING_POINT 8

using mvmcts::ObjectiveVec;
using mvmcts::ThresUCTStatistic;

template <typename T>
class GridWorldSuite : public ::testing::Test {
 protected:
  GridWorldSuite() {}
  History run(const ObjectiveVec& thres) {
    auto test_runner = std::make_unique<TestRunner>(
        std::make_shared<CrossingTestEnv<T>>(thres));
    auto res = test_runner->RunTest(3000, 16);
    TestRunner::Result::WriteHeader(LOG(INFO));
    LOG(INFO) << res;
    return test_runner->GetLatestTestEnv()->state_history;
  }
};
using testing::Types;

typedef Types<ThresUCTStatistic, ThresGreedyStatistic> MyTypes;
TYPED_TEST_SUITE(GridWorldSuite, MyTypes);


TYPED_TEST(GridWorldSuite, collision) {
  ObjectiveVec threshold(ObjectiveVec::Zero(3));
  threshold << -1.0, -1.0, std::numeric_limits<ObjectiveVec::Scalar>::max();
  auto hist = this->run(threshold);
  // Collision of 0 and 2
  EXPECT_EQ(hist.back()(0), hist.back()(2));
  // 1 has passed the merging point
  EXPECT_GT(hist.back()(1), MERGING_POINT);
}

TYPED_TEST(GridWorldSuite, pass) {
  ObjectiveVec threshold(ObjectiveVec::Zero(3));
  threshold << -0.37, -0.37, std::numeric_limits<ObjectiveVec::Scalar>::max();
  auto hist = this->run(threshold);
  // Everyone should have passe the merging point
  EXPECT_GT(hist.back()(0), MERGING_POINT);
  EXPECT_GT(hist.back()(1), MERGING_POINT);
  EXPECT_GT(hist.back()(2), MERGING_POINT);
  // Order should be 1,2,0
  EXPECT_LT(hist.back()(0), hist.back()(2));
  EXPECT_LT(hist.back()(2), hist.back()(1));
}

TYPED_TEST(GridWorldSuite, livelock) {
  ObjectiveVec threshold(ObjectiveVec::Zero(3));
  threshold << -0.1, -0.8, std::numeric_limits<ObjectiveVec::Scalar>::max();
  auto hist = this->run(threshold);
  // Only 1 should have passed the merging point
  EXPECT_LT(hist.back()(0), MERGING_POINT);
  EXPECT_GT(hist.back()(1), MERGING_POINT);
  EXPECT_LT(hist.back()(2), MERGING_POINT);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_v = 1;
  FLAGS_alsologtostderr = true;
  mvmcts::RandomGenerator::random_generator_ = std::mt19937(1000);
  return RUN_ALL_TESTS();
}
