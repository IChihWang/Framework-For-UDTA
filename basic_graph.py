import itertools
import numpy as np


class Node:
    newid = itertools.count(0)

    def __init__(self):
        self.id = next(self.newid)
        self.is_activated = False
        self.in_links = []  # [link..]
        self.out_links = [] # touple: [](turn, link)..]
        self.link_to_turn = dict()

        # For Dijkstra's algorithm
        self.arrival_time = float('inf')  # For Dijkstra node value
        self.is_visited = False
        self.from_link = None

        # Variable for connecting to intersection manager
        self.connect_to_intersection = None
        self.in_intersection_lane = None



    # Methods for Dijkstra's algorithm
    def initial_for_dijkstra(self):
        self.arrival_time = float('inf')
        self.is_visited = False
        self.from_link = None

    def set_connect_to_intersection(self, intersection):
        self.connect_to_intersection = intersection
    def get_connect_to_intersection(self):
        return self.connect_to_intersection

    def set_in_intersection_lane(self, lane):
        self.in_intersection_lane = lane
    def get_in_intersection_lane(self):
        return self.in_intersection_lane

    def set_is_visited(self, is_visited):
        self.is_visited = is_visited
    def get_is_visited(self):
        return self.is_visited

    def set_arrival_time(self, arrival_time):
        self.arrival_time = arrival_time
    def get_arrival_time(self):
        return self.arrival_time

    def set_from_link(self, from_link):
        self.from_link = from_link
    def get_from_link(self):
        return self.from_link
    def get_from_node(self):
        if self.from_link == None:
            return None
        else:
            return self.from_link.in_node

    def add_out_links(self, turn, new_link):
        self.out_links.append((turn,new_link))
        self.link_to_turn[new_link] = turn
    def get_turn_from_link(self, link):
        return self.link_to_turn[link]




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
        # self.traveling_time = list(np.random.uniform(low=1, high=3, size=20))
        #self.traveling_time = [0 for _ in range(5)]
        #TODO: change the traveling time
        #self.traveling_time = 0 # Constant, traveling time in maximum speed
        self.traveling_time = 0
        self.delay = [0 for _ in range(5)]
        self.car_data_base = [[] for _ in range(5)] # 2D array
        self.at = []

    '''
    def updateCost(self, value):
        self.cost = value

    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.id)
    '''


class Car:
    #new_id = itertools.count(0)

    def __init__(self):
        self.id = None
        self.arriving_time = None
        self.position = None
        self.turning = None
        self.src_node = None
        self.dst_node = None
        self.path_node = []
        self.path_node_at_idx = dict()
        self.path_link_delay = dict()
        self.path_link = []
        self.is_scheduled = None
        self.traveling_time = None
