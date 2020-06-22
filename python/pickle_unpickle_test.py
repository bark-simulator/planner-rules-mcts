import unittest
import pickle
import numpy as np

# note: run this test with bazel test //python:pickle_unpickle_test --define planner_uct=true
# it uses the bark python module


from bark.core.models.behavior import *
from bark.core.models.dynamic import *
from bark.core.world.prediction import PredictionSettings
from bark.runtime.commons import ParameterServer

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

        behavior = BehaviorUCTMultiAgent(params, PredictionSettings(BehaviorMPMacroActions(params), None, None, []), [], [], {})

        behavior_after = pickle_unpickle(behavior)

        self.assertTrue(isinstance(behavior_after, BehaviorUCTMultiAgent))


if __name__ == '__main__':
    unittest.main()