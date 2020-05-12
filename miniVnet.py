 #coding=utf-8

import math
from basic_graph import Car
from vehicle_map import Intersection, Road, Sink
import itertools
import heapq
import global_val

class MiniVnet:
    def __init__(self):
        self.is_compiled = False
        self.intersections = []
        self.roads = []
        self.sinks = []
        self.nodes = []
        self.dgraph = {}
        self.intersection_nodes = []
        self.sink_nodes = []
        self.car_number = 0

    # ================ Setup the network ====================
    def addIntersection(self, name, num_lane):
        assert self.is_compiled == False, "The miniVnet has been compiled"

        new_intersection = Intersection(name, num_lane)
        self.intersections.append(new_intersection)
        return new_intersection

    def addSink(self, name, num_lane):
        assert self.is_compiled == False, "The miniVnet has been compiled"

        new_sink = Sink(name, num_lane)
        self.sinks.append(new_sink)
        return new_sink

    def connect(self, component_1, idx_1, component_2, idx_2):
        assert self.is_compiled == False, "The miniVnet has been compiled"
        assert component_1.num_lane == component_2.num_lane, "Lane number mismatch: " + str(
            component_1.name) + " " + str(component_2.name)

        num_lane = component_1.num_lane

        new_road = Road(num_lane)
        self.roads.append(new_road)
        component_1.connect(idx_1, new_road)
        component_2.connect(idx_2, new_road)

    def createGridNetwork(self, N, num_lane):
        intersections = [[self.addIntersection('I' + str(i) + '_' + str(j), num_lane) for j in range(N)] for i in
                         range(N)]

        sinks = [[self.addSink('S' + str(i) + '_' + str(j), num_lane) for j in range(N)] for i in range(4)]

        # Connect intersections
        for i_row in range(N):
            for i_col in range(1, N):
                component_1 = intersections[i_row][i_col - 1]
                component_2 = intersections[i_row][i_col]
                self.connect(component_1, 1, component_2, 3)
        for i_col in range(N):
            for i_row in range(1, N):
                component_1 = intersections[i_row - 1][i_col]
                component_2 = intersections[i_row][i_col]
                self.connect(component_1, 0, component_2, 2)

        # Connect sinks
        for i_idx in range(N):
            target_sink = sinks[0][i_idx]
            target_intersection = intersections[0][i_idx]
            self.connect(target_sink, 0, target_intersection, 2)

            target_sink = sinks[1][i_idx]
            target_intersection = intersections[i_idx][N - 1]
            self.connect(target_sink, 0, target_intersection, 1)

            target_sink = sinks[2][i_idx]
            target_intersection = intersections[N - 1][N - 1 - i_idx]
            self.connect(target_sink, 0, target_intersection, 0)

            target_sink = sinks[3][i_idx]
            target_intersection = intersections[N - 1 - i_idx][0]
            self.connect(target_sink, 0, target_intersection, 3)

        self.compile()

        return self

    # ==================== Routing For Cars ======================= #
    # Debug Purpose
    def debug(self):
        pass


    # ======================== Compile ========================== #
    def compile(self):
        self.is_compiled = True

    # ==================== UDTA functions ==#==================== #

    def udta(self, cars):
        # TODO: convert car's current position to src_node & bias
        '''
        for car in cars:
            self.dijkstra(car, src_node, car.dst_node, bias):
        '''
        # self.dijkstra(car)
        # self.update_map(car)

    # =========== Use the DataStructure to route ====================== #
    # Important: Not thread safe!! only allow one car routing at a time (because data structure)
    def dijkstra(self, car, src_node, dst_node, time_bias):
        # time_bias is the time bias for those on-the-fly cars, whose is not yet at the source node

        # (initialization)  set every node to be inf. Except the start node= 0
        for intersection in self.intersections:
            intersection.initial_for_dijkstra()
        for sink in self.sinks:
            sink.initial_for_dijkstra()

        src_node.set_arrival_time(0 + time_bias)

        unvisited_queue = [(src_node.get_arrival_time(), src_node)]

        while len(unvisited_queue) > 0:
            current_arrival_time, current_node = heapq.heappop(unvisited_queue)

            # Only process the first time we pop the node from the queue
            if current_node.get_is_visited() == True:
                continue
            else:
                current_node.set_is_visited(True)

            # Update link cost by Roadrunner
            # update when the car is on the road, store the result within the intersection
            # Be aware of the time stemp!!!
            if len(current_node.out_links) > 0:
                turning, out_link = current_node.out_links[0]
                next_node = out_link.out_node

                next_intersection = next_node.get_connect_to_intersection()
                if next_intersection != None:
                    next_intersection.get_cost_from_manager(current_arrival_time, current_node)

            # visiting neighbors
            for turning, out_link in current_node.out_links:
                neighbor_node = out_link.out_node
                current_time = current_node.get_arrival_time()

                # 1. Check if there is a record of the traveling_time
                if len(out_link.delay)-1 < int(math.floor(current_time)):
                    # Append zeros as the traveling_time into the data_list
                    for _ in range(int(math.floor(current_time))-(len(out_link.delay)-1)):
                        out_link.delay.append(0)
                        # Future work: append something other than zeros
                else:
                    pass

                # 2. Check the value of the neighbors in Dijkstra
                arriving_neighbor_time = current_time +  out_link.traveling_time + out_link.delay[int(math.floor(current_time))]
                if neighbor_node.get_arrival_time() > arriving_neighbor_time:
                    neighbor_node.set_arrival_time(arriving_neighbor_time)
                    neighbor_node.set_from_link(out_link)
                    heapq.heappush(unvisited_queue, (arriving_neighbor_time, neighbor_node))

        #'''
        for intersection in self.intersections:
            intersection.print_node_arrival_time()
        print("============================")
        for sink in self.sinks:
            sink.print_node_arrival_time()
        #'''

        # Trace back from destination to source
        tracing_node = dst_node
        car.path_node.insert(0, tracing_node)
        tracing_link = tracing_node.get_from_link()

        while tracing_link != None:
            tracing_node = tracing_node.get_from_node()
            car.path_node.insert(0, tracing_node)
            car.path_link.insert(0, tracing_link)
            tracing_link = tracing_node.get_from_link()

        assert car.path_link > 0, "The car cannot find the route"
        # TODO: if rerouting: check if it increase global cost
        #       if not: the route must be taken
        for node in car.path_node:
            print(node, node.arrival_time)

        for link in car.path_link:
            print(link, link.id)

    # =========== Update the cost with given path ====================== #
    def update_map(self, car):
        # the time_bias is included during routing

        # path: in_node -----> (link) -----> out_node -----> (next_link)
        for link_idx in range(len(car.path_link)-1):
            link = car.path_link[link_idx]
            next_link = car.path_link[link_idx+1]
            in_node = car.path_node[link_idx]
            out_node = car.path_node[link_idx+1]

            # Only store the car info in the "road" (connected to the intersection)
            # next_link: links inside the intersection
            if out_node.get_connect_to_intersection() != None:
                # Get the turns from the node
                turn = out_node.get_turn_from_link(next_link)

                # compute the position on the link
                enter_link_time = in_node.get_arrival_time()
                enter_link_time_idx = int(math.floor(enter_link_time))
                init_position = (in_node.get_arrival_time() - enter_link_time_idx) * global_val.MAX_SPEED

                # compute the traveling time
                delay = next_link.delay[enter_link_time_idx]
                actual_travel_time = link.traveling_time + delay

                # expend the list size if not enough
                if len(link.car_data_base) - 1 < enter_link_time_idx:
                    for _ in range(enter_link_time_idx-(len(link.car_data_base) - 1)):
                        link.car_data_base.append([])

                print("==========")
                print(enter_link_time)
                # 1. add the car into the database "unscheduled"
                unscheduled_copy_car = Car()
                unscheduled_copy_car.id = car.id
                unscheduled_copy_car.position = init_position
                unscheduled_copy_car.turning = turn
                unscheduled_copy_car.is_scheduled = False
                link.car_data_base[enter_link_time_idx] = unscheduled_copy_car


                # 2. add the future car into the database "scheduled"
                current_time_step = 1
                remaining_actual_travel_time = actual_travel_time - current_time_step
                while remaining_actual_travel_time >= 0:
                    current_time_idx = int(math.floor(enter_link_time + current_time_step))
                    if len(link.car_data_base) - 1 < current_time_idx:
                        link.car_data_base.append([])

                    scheduled_copy_car = Car()
                    scheduled_copy_car.id = car.id
                    scheduled_copy_car.arriving_time = remaining_actual_travel_time
                    scheduled_copy_car.turning = turn
                    scheduled_copy_car.is_scheduled = True
                    link.car_data_base[current_time_idx] = scheduled_copy_car

                    current_time_step = current_time_step+1
                    remaining_actual_travel_time = actual_travel_time - current_time_step

                

    '''

    # =========== Use the DataStructure to route ====================== #
    def getNode(self, out_edge):
        return out_edge.out_node

    # Get the Intersection number from the number of the node number for finding the intersection to update cost value
    def sink_to_node(self, sink):
        sink_node_list = []
        for nodes in sink.node_groups:
            for sink_node in nodes:
                sink_node_list.append(sink_node)
        return sink_node_list

    def node_to_intersetion(self, node):
        for intersection in self.intersections:
            for j in range(len(intersection.direction_nodes)):
                inodes = intersection.direction_nodes[j]
                for inode in inodes:
                    for insec_node in inode:
                        if node is insec_node:
                            return intersection

    def dijkstra(self, s1, s2, global_time):
        # Each dijkstra create a new car id
        self.car_number += 1
        car_id = self.car_number + 1

        self.start = self.sinks[s1]
        self.goal = self.sinks[s2]
        print('--------- Start Routing ---------')
        print('Routing the path goes from {} to {}'.format(self.start, self.goal))
        self.potential_list = []
        self.from_node = {}
        self.from_edge = {}
        self.car_timestamp = {}

        # (initialization)  set every node to be inf. Except the start node= 0
        for node in self.nodes:
            node.setValue(float('inf'))

        start_nodes = self.sink_to_node(self.start)
        for node in start_nodes:
            node.setValue(0)

        # (initialization) Add the source node into the queue
        self.node_list = []
        for snode in start_nodes:
            self.node_list.append(snode)

        # Dijkstra. Check the dictionary is empty or note
        # TODO: change the list to heap
        while self.node_list:
            # Greedy. Loeset node for this
            node = self.node_list.pop()
            time = node.value


            for out_edge in node.out_links:
                out_node = self.getNode(out_edge)
                for _ in range(math.ceil(time - 9)):
                    # append a constant value: 1
                    out_edge.cost.append(0)
                self.next_time = node.value + out_edge.cost[math.ceil(time)]
                if self.next_time < out_node.value:
                    out_node.setValue(self.next_time)
                    self.node_list.append(out_node)
                    self.from_node[out_node] = node
                    self.from_edge[out_node] = out_edge
            self.car_timestamp[node] = node.value

        # Record the path
        self.goal_nodes = self.sink_to_node(self.goal)
        self.path = []
        self.path_node_list = []
        self.path_edge_list = []
        for gnode in self.goal_nodes:
            if gnode in self.from_node:
                self.path_node_list.append(gnode)
                while self.path_node_list:
                    try:
                        self.path_node = self.path_node_list.pop()
                        self.path.insert(0, self.path_node)
                        self.path_node_list.append(self.from_node[self.path_node])
                        self.path_edge_list.insert(0, self.from_edge[self.path_node])
                    except KeyError:
                        break
        for node in self.path:
            self.potential_list.append(self.car_timestamp[node])
        minTime = self.potential_list[-1]
        t = len(self.potential_list)
        for x in range(len(self.potential_list) - 1):
            if (self.potential_list[x + 1] == 0) & (self.potential_list[x] <= minTime):
                minTime = self.potential_list[x]
                t = x
        self.optimaltime = []
        self.optimallist = []
        while self.potential_list[t - 1] != 0:
            self.optimallist.insert(0, self.path[t - 1])
            self.optimaltime.insert(0, self.potential_list[t - 1])
            t = t - 1
        self.optimallist.insert(0, self.path[t - 1])
        self.optimaltime.insert(0, self.potential_list[t - 1])
        # print(self.path, self.potential_list)
        self.print_list = []
        self.print_list.append(self.start)
        for node in self.optimallist[1:-1]:
            self.print_list.append(self.node_to_intersetion(node))
        self.print_list.append(self.goal)

        # calculate AT
        time_offset = global_time
        for i in range(1, len(self.optimaltime)):
            # 向上取整
            remaining_time = math.ceil(self.optimaltime[i])
            for j in range(time_offset):
                if len(self.path_edge_list[i - 1].at) <= j:
                    self.path_edge_list[i - 1].at.append(None)

            for j in range(time_offset, time_offset + remaining_time):
                if len(self.path_edge_list[i - 1].at) <= j:
                    self.path_edge_list[i - 1].at.append(None)
                self.path_edge_list[i - 1].at[j].append(Car(car_id, time_offset + remaining_time - j, None))

            for k in range(time_offset):
                self.path_edge_list[i - 1].at[k].append(
                    Car(car_id, None, 3 * (k + math.ceil(self.optimallist[i - 1].value) - self.optimallist[i - 1].value)))

            time_offset = time_offset + remaining_time
        # TODO: car lane update

        print('The car is ' + str(car_id))
        print('The path is ' + str(self.print_list))
        print('Coresponding time is' + str(self.optimaltime))
        print('The detailed link is' + str(self.path_edge_list))

        for link in self.path_edge_list:
            print(link.at)
        print('# ============================= #')

        return self.optimallist, self.print_list, self.optimaltime
        #     self.path.insert(0,start)
        #     if goal_node.value != infinity:
        #     self.potential_list = self.potential_list[::-1]
        #     print(self.potential_list)
        # return self.path, self.potential_list

    # ======================= UPDATE The Intersection Part ===============================#




    def updateLinkCost(self, path, time):
        for i in range(0, len(path) - 1, 2):
            for link in path[i].out_links:
                if link.out_node is path[i + 1]:
                    link.cost[time[i]] += 1

    def resetLinkCost(self, path, time):
        for i in range(0, len(path) - 1, 2):
            for link in path[i].out_links:
                if link.out_node is path[i + 1]:
                    link.cost[time[i]] -= 1

    def updateIntersection(self, path, time):
        intersection_list = self.intersection_nodes
        self.timelist = {}
        for i in range(len(path)):
            node = path[i]
            # print(node.value)
            if node in intersection_list:
                self.timelist[time[i]] = self.node_to_intersetion(node)
        return self.timelist

    def mergeDict(self, dict1, dict2):
        dict3 = {**dict1, **dict2}
        for key, value in dict3.items():
            if key in dict1 and key in dict2:
                dict3[key] = [value, dict1[key]]
        return dict3


    # Merge dictionaries and add values of common keys in a list
    def multipleIntCount(self, paths, times):
        time_dic = {}
        for i in range(len(paths)):
            # time_dic.update(self.updateIntersection(paths[i],times[i]))
            time_dic = self.mergeDict(time_dic, self.updateIntersection(paths[i], times[i]))
        time_dic = dict(sorted(time_dic.items()))
        print(time_dic)

    # ============= Add Links to the Network ====================== #
    def appendLinktoIntersection(self):
        for graph_intersection in self.intersections:
            # Add link inside Intersections
            for i in range(4):
                for j in range(graph_intersection.num_lane):
                    for k in range(3):
                        graph_intersection.new_link[i][j][k].in_node.out_links.append(
                            graph_intersection.new_link[i][j][k])

            for j in range(len(graph_intersection.direction_nodes)):
                inodes = graph_intersection.direction_nodes[j]
                for jj in range(len(inodes)):
                    inode = inodes[jj]
                    for insec_node in inode:
                        for graph_road in self.roads:
                            linkg = graph_road.link_groups
                            for l in range(2):
                                for link in linkg[l]:
                                    if link.in_node is insec_node:
                                        link.in_node.out_links.append(link)

    def appendLinktoSink(self):
        for sink_node in self.sink_nodes:
            for graph_road in self.roads:
                linkg = graph_road.link_groups
                for l in range(2):
                    for link in linkg[l]:
                        if link.in_node is sink_node:
                            link.out_node.in_links.append(link)
                            sink_node.out_links.append(link)
                        elif link.out_node is sink_node:
                            link.in_node.out_links.append(link)
                            sink_node.in_links.append(link)

    # ======================== Compile ========================== #
    def compile(self):
        self.is_compiled = True
        # Network list
        for sink in self.sinks:
            for nodes in sink.node_groups:
                for sink_node in nodes:
                    self.sink_nodes.append(sink_node)
        for intersection in self.intersections:
            for j in range(len(intersection.direction_nodes)):
                inodes = intersection.direction_nodes[j]
                for inode in inodes:
                    for insec_node in inode:
                        self.intersection_nodes.append(insec_node)
        for graph_road in self.roads:
            linkg = graph_road.link_groups
            for l in range(2):
                for link in linkg[l]:
                    self.nodes.append(link.in_node)
                    self.nodes.append(link.out_node)
        self.nodes = list(set(self.nodes))
        self.appendLinktoSink()
        self.appendLinktoIntersection()

    # ================ Run the network ===================


    '''
