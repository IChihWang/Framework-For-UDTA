
import itertools
import numpy as np

np.random.seed(0)

class NODE:
    newid = itertools.count(0) 
    def __init__(self):
        self.id = next(self.newid)
        self.is_activated = False
        self.in_links = []
        self.out_links = []
        self.value = None # For Dijkstra node value
    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.id)


class LINK:
    newid = itertools.count(0)
    def __init__(self):
        self.id = next(self.newid)
        self.in_node = None
        self.out_node = None
        self.cost = [int(np.random.randint(10, size=1)) for _ in range(10)]
    def updateCost(self,value):
        self.cost = value
    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.id)
