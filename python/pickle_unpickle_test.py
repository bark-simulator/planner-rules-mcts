import unittest
import pickle
import numpy as np

# note: run this test with bazel test //python:pickle_unpickle_test --define planner_uct=true
# it uses the bark python module


from bark.models.behavior import BehaviorMCTSMultiAgent
from modules.runtime.commons.parameters import ParameterServer

def pickle_unpickle(object):
    with open('temp.pickle','wb') as f:
        pickle.dump(object,f)
    object = None
    with open( 'temp.pickle', "rb" ) as f:
        object = pickle.load(f)
    return object


class PickleTests(unittest.TestCase):
    def test_behavior_uct_single_agent_pickle(self):
        params = ParameterServer()
        params["BehaviorMCTSAgent"]["Rules"]["common"]["formula"] = "G !collision_ego"
        params["BehaviorMCTSAgent"]["Rules"]["common"]["weight"] = -700.0
        params["BehaviorMCTSAgent"]["Rules"]["common"]["priority"] = 0

        behavior = BehaviorMCTSMultiAgent(params)

        behavior_after = pickle_unpickle(behavior)

        self.assertTrue(isinstance(behavior_after, BehaviorMCTSMultiAgent))


if __name__ == '__main__':
    unittest.main()