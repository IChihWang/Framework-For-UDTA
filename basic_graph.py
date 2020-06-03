import itertools
import numpy as np


class Node:
    def __init__(self, id):
        self.id = id
        self.is_activated = False
        self.in_links = []  # [link..]
        self.out_links = [] # touple: [](turn, link)..]
        self.link_to_turn = dict()


        # Variable for connecting to intersection manager
        self.connect_to_intersection = None
        self.in_intersection_lane = None



    def set_connect_to_intersection(self, intersection):
        self.connect_to_intersection = intersection
    def get_connect_to_intersection(self):
        return self.connect_to_intersection

    def set_in_intersection_lane(self, lane):
        self.in_intersection_lane = lane
    def get_in_intersection_lane(self):
        return self.in_intersection_lane


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
        #self.traveling_time = np.random.uniform(low=1, high=10)
        self.traveling_time = 0.5
        self.delay = [0 for _ in range(5)]
        self.car_data_base = [[] for _ in range(5)] # 2D array
        self.length = 0

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
        self.length = 5 # to-be-modified

        # For routing
        self.path_node = []
        self.path_link = []
        self.recorded_in_database = dict()  # {"link":link, "time_car":[(time, car)]}
        self.is_scheduled = None

        # For recording in the database
        self.arriving_time = None
        self.position = None
        self.turning = None
        self.lane = None
