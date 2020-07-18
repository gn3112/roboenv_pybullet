from collections import deque
from random import randint

class ClassicBuffer(object):
    def __init__(self, buffer_size=100000):
        self.d = deque(maxlen=buffer_size)
    def add(self, transition):
        self.d.append(transition)
    def sample(self, n_samples):
        #TODO: Compare to sampling from numpy array
        a = []
        for i in range(n_samples):
            idx = randint(0,len(self.d)-1)
            a = a + self.d[idx]
        return a

class PERBuffer(object):
    pass

class HERBuffer(object):
    pass
