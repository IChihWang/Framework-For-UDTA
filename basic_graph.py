import itertools
import numpy as np


class Node:
    newid = itertools.count(0)

    def __init__(self):
        self.id = next(self.newid)
        self.is_activated = False
        self.in_links = []
        self.out_links = []
        self.value = float('inf')  # For Dijkstra node value

    def set_value(self, value):
        self.value = value

    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.id)


class Link:
    newid = itertools.count(0)

    def __init__(self):
        self.id = next(self.newid)
        self.in_node = None
        self.out_node = None
        # float
        # self.cost = [int(np.random.randint(10, size=1)) for _ in range(10)]
        self.cost = list(np.random.uniform(low=0.1, high=10, size=10))
        self.at = []

    def updateCost(self, value):
        self.cost = value

    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.id)


class Car:
    #new_id = itertools.count(0)

    def __init__(self, car_id, AT, position):
        self.id = car_id
        self.AT = AT
        self.position = position
        self.turning = None
        self.path = []
        self.is_scheduled = 0
