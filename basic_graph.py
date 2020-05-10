import itertools
import numpy as np


class Node:
    newid = itertools.count(0)

    def __init__(self):
        self.id = next(self.newid)
        self.is_activated = False
        self.in_links = []  # [link..]
        self.out_links = [] # touple: [](turn, link)..]

        # For Dijkstra's algorithm
        self.arrival_time = float('inf')  # For Dijkstra node value
        self.is_visited = False
        self.from_link = None

    # Methods for Dijkstra's algorithm
    def initial_for_dijkstra(self):
        self.arrival_time = float('inf')
        self.is_visited = False
        self.from_link = None

    def set_is_visited(self, is_visited):
        self.is_visited = is_visited

    def set_arrival_time(self, arrival_time):
        self.arrival_time = arrival_time

    def get_arrival_time(self):
        return self.arrival_time

    def set_from_link(self, from_link):
        self.from_link = from_link

    def get_from_link(self):
        return self.from_link

    def get_from_node(self):
        return self.from_link.in_node




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
        self.traveling_time = list(np.random.uniform(low=1, high=3, size=20))
        self.at = []

    '''
    def updateCost(self, value):
        self.cost = value

    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.id)
    '''


class Car:
    #new_id = itertools.count(0)

    def __init__(self, car_id, AT, position):
        self.id = car_id
        self.AT = AT
        self.position = position
        self.turning = None
        self.path = []
        self.is_scheduled = 0
