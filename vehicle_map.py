 #coding=utf-8

import math

from basic_graph import Node, Link, Car
import itertools
import global_val

class Intersection:
    def __init__(self, name, num_lane):
        self.name = name
        self.num_lane = num_lane
        self.components = [None for i in range(4)]
        self.in_nodes = [Node(str(name)+"_i_"+str(i)) for i in range(4)]
        self.out_nodes = [Node(str(name)+"_o_"+str(i)) for i in range(4)]
        #self.direction_nodes = [[[Node() for k in range(num_lane)] for j in range(2)] for i in range(4)]
        self.links = []
        #self.new_link = [[[Link() for k in range(3)] for j in range(num_lane)] for i in range(4)]

        # Allocate links to nodes
        for i in range(4):
            for k in range(3):  # left, straight, right
                new_link = Link()

                source_node = self.in_nodes[i]
                source_node.set_connect_to_intersection(self)
                source_node.set_in_intersection_lane(i * num_lane)
                source_node.add_out_links(k, new_link)
                new_link.in_node = source_node

                sink_node = self.out_nodes[(i-1-k)%4]
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

    def initial_for_dijkstra(self, nodes_arrival_time, nodes_from_link, nodes_is_visit):
        for i in range(4):
            nodes_arrival_time[self.in_nodes[i].id] = float('inf')
            nodes_from_link[self.in_nodes[i].id] = None
            nodes_is_visit[self.in_nodes[i].id] = False

            nodes_arrival_time[self.out_nodes[i].id] = float('inf')
            nodes_from_link[self.out_nodes[i].id] = None
            nodes_is_visit[self.out_nodes[i].id] = False

    def get_cost_from_manager(self, arrival_time, node):
        # Call single intersection manager
        # Update the travel time (delay) on links within the intersection
        node.get_in_intersection_lane()
        # TODO: build a "new car"
        #for in_node in self.in_nodes:


        pass

    def print_node_arrival_time(self):
        # For debugging
        print("=== ", self.name)
        for i in range(4):
            print("  ", self.in_nodes[i].id, self.in_nodes[i].arrival_time)
            print("  ", self.out_nodes[i].id, self.out_nodes[i].arrival_time)


    def print_details(self):
        print(self.name, self.num_lane, self.components, self.links)





    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.name)


class Road:
    def __init__(self, num_lane):
        self.num_lane = num_lane
        self.link_groups = [Link() for i in range(2)]
        self.components = [None for i in range(2)]
        self.length = global_val.ROAD_LENGTH

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
            self.link_groups[0].in_node = in_nodes
            self.link_groups[1].out_node = out_nodes

            in_nodes.add_out_links(global_val.STRAIGHT_TURN, self.link_groups[0])
            out_nodes.in_links.append(self.link_groups[1])
        else:
            self.components[1] = component
            self.link_groups[1].in_node = in_nodes
            self.link_groups[0].out_node = out_nodes

            in_nodes.add_out_links(global_val.STRAIGHT_TURN, self.link_groups[1])
            out_nodes.in_links.append(self.link_groups[0])


#    def checkSetting(self):
#        component_list = [component for component in self.components if component != None]
#
#      assert len(component_list) == 2, "Error: Road " + str(self.name) + " has too few connections."


class Sink:
    '''
    Sink                  Road
    ========= | ----------------------------
    in_node  ====     <- link group -
          --- | ----------------------------
    out_node ====     - link group ->
    ========= | ----------------------------
    '''

    newid = itertools.count(0)

    def __init__(self, name, num_lane):
        self.name = name
        self.num_lane = num_lane
        #self.node_groups = [[Node() for i in range(num_lane)] for i in range(2)]
        self.in_nodes = Node(str(name)+"_i_0")
        self.out_nodes = Node(str(name)+"_o_0")
        self.components = None
        self.id = next(Sink.newid)


    def print_details(self):
        print(self.name, self.num_lane, self.components, self.in_nodes, self.out_nodes)

    def connect(self, index, road):
        assert self.components == None, "The sink is already connected, sink: " + str(self.name)

        assert index == 0, "Index out of range, sink: " + str(self.name)

        self.components = road
        road.connect(self, self.out_nodes, self.in_nodes)


    def initial_for_dijkstra(self, nodes_arrival_time, nodes_from_link, nodes_is_visit):
        nodes_arrival_time[self.in_nodes.id] = float('inf')
        nodes_from_link[self.in_nodes.id] = None
        nodes_is_visit[self.in_nodes.id] = False

        nodes_arrival_time[self.out_nodes.id] = float('inf')
        nodes_from_link[self.out_nodes.id] = None
        nodes_is_visit[self.out_nodes.id] = False

    def print_node_arrival_time(self):
        # For debugging
        print("=== ", self.name)
        print("  ", self.in_nodes.id, self.in_nodes.arrival_time)
        print("  ", self.out_nodes.id, self.out_nodes.arrival_time)


    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.name)

    def checkSetting(self):
        assert self.components != None, "Error: sink " + str(self.name) + " has too few connections."
