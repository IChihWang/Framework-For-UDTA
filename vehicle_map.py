 #coding=utf-8

import math

from basic_graph import Node, Link, Car
import itertools


class Intersection:
    def __init__(self, name, num_lane):
        self.name = name
        self.num_lane = num_lane
        self.components = [None for i in range(4)]
        self.in_nodes = [[Node() for k in range(num_lane)] for i in range(4)]
        self.out_nodes = [[Node() for k in range(num_lane)] for i in range(4)]
        #self.direction_nodes = [[[Node() for k in range(num_lane)] for j in range(2)] for i in range(4)]
        self.links = []
        #self.new_link = [[[Link() for k in range(3)] for j in range(num_lane)] for i in range(4)]

        # Allocate links to nodes
        for i in range(4):
            for j in range(num_lane):
                for k in range(3):  # left, straight, right
                    source_node = self.in_nodes[i][j]
                    new_link = Link()
                    source_node.out_links.append(new_link)
                    new_link.in_node = source_node

                    sink_node = self.out_nodes[(i-1-k)%4][j]
                    sink_node.in_links.append(new_link)
                    new_link.out_node = sink_node

                    self.links.append(new_link)



    def connect(self, index, road):
        assert self.components[
                   index] == None, "Two components are assigned to the same entry of the intersection: " + str(
            self.name)

        assert index < 4, "Index out of range, intersection: " + str(self.name)

        self.components[index] = road

        road.connect(self, self.out_nodes[index], self.in_nodes[index])

    def checkSetting(self):
        component_list = [component for component in self.components if component != None]

        assert len(component_list) > 2, "Error: intersection " + str(self.name) + " has too few connections."

    def set_all_node_value(self, val):
        for i in range(4):
            for j in range(self.num_lane):
                self.in_nodes[i][j].set_value(val)
                self.out_nodes[i][j].set_value(val)

    def print_details(self):
        print(self.name, self.num_lane, self.components, self.links)





    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.name)


class Road:
    def __init__(self, num_lane):
        self.num_lane = num_lane
        self.link_groups = [[Link() for j in range(num_lane)] for i in range(2)]
        self.components = [None for i in range(2)]

    '''
    ------------------------
      <- link group [0] ->
    ------------------------
      <- link group [1] ->
    ------------------------
    '''

    def connect(self, component, in_nodes, out_nodes):
        assert self.components[0] == None or self.components[1] == None, "A road is overly assigned"

        if self.components[0] == None:
            self.components[0] = component
            for i in range(self.num_lane):
                self.link_groups[0][i].in_node = in_nodes[i]
                self.link_groups[1][i].out_node = out_nodes[i]
        else:
            self.components[1] = component
            for i in range(self.num_lane):
                self.link_groups[1][i].in_node = in_nodes[i]
                self.link_groups[0][i].out_node = out_nodes[i]


#    def checkSetting(self):
#        component_list = [component for component in self.components if component != None]
#
#      assert len(component_list) == 2, "Error: Road " + str(self.name) + " has too few connections."


class Sink:
    '''
    Sink                  Road
    ========= | ----------------------------
    in_node  ====     <- link group ->
          --- | ----------------------------
    out_node ====     <- link group ->
    ========= | ----------------------------
    '''

    newid = itertools.count(0)

    def __init__(self, name, num_lane):
        self.name = name
        self.num_lane = num_lane
        #self.node_groups = [[Node() for i in range(num_lane)] for i in range(2)]
        self.in_node = [Node() for i in range(num_lane)]
        self.out_node = [Node() for i in range(num_lane)]
        self.components = None
        self.id = next(Sink.newid)


    def print_details(self):
        print(self.name, self.num_lane, self.components, self.in_node, self.out_node)

    def connect(self, index, road):
        assert self.components == None, "The sink is already connected, sink: " + str(self.name)

        assert index == 0, "Index out of range, sink: " + str(self.name)

        self.components = road

        road.connect(self, self.in_node, self.out_node)

    def set_all_node_value(self, val):
        for i in range(self.num_lane):
            self.in_nodes[i].set_value(val)
            self.out_nodes[i].set_value(val)

    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.name)

    def checkSetting(self):
        assert self.components != None, "Error: sink " + str(self.name) + " has too few connections."
