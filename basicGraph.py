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
        self.value = float('inf')  # For Dijkstra node value

    def setValue(self, value):
        self.value = value

    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.id)


class LINK:
    newid = itertools.count(0)

    def __init__(self):
        self.id = next(self.newid)
        self.in_node = None
        self.out_node = None
        #self.cost = [int(np.random.randint(10, size=1)) for _ in range(10)]
        self.cost = 0
        self.at = []

    def updateCost(self, value):
        self.cost = value

    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.id)


class CAR:
    newid = itertools.count(0)

    def __init__(self):
        self.id = next(self.newid)
        self.AT = {}
        self.position = None
        self.turning = None
        self.path = []
        self.is_scheduled = 0
