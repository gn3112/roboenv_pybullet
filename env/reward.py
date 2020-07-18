import numpy as np
from math import sqrt

class _Reward(object):
    # TODO: Define supported tasks 

    # Define types for args
    def __init__(self, threshold, final_reward):
        self.threshold = threshold
    
    def cartesian_dist(self, ee_pos, target_pos):
        dist = sqrt(np.sum(np.square(np.array(ee_pos)-np.array(target_pos))))
        return dist

class Sparse(_Reward):
    
    def __init__(self, threshold=0.01, final_reward=1, intermediate_reward=0):
        super(Sparse, self).__init__(threshold, final_reward)
        self.intermediate_reward = intermediate_reward

    def __call__(self, ee_pos, target_pos):
        if self.cartesian_dist(ee_pos, target_pos) < self.threshold:
            return self.final_reward
        else:
            return self.intermediate_reward

class Dense(_Reward):

    def __init__(self, threshold=0.01, final_reward=1):
        super(Dense, self).__init__(threshold=threshold, final_reward=final_reward)

    def dense(self, action, state):
        criterion = self.cartesian_dist(ee_pos, target_pos)
        if criterion < threshold:
            return self.final_reward
        else:
            return -criterion