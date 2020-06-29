 #coding=utf-8

import math
from basic_graph import Car
from vehicle_map import Intersection, Road, Sink
import itertools
import heapq
import global_val
import numpy
import scipy.io as io

class MiniVnet:
    def __init__(self):
        self.is_compiled = False
        self.intersections = dict()
        self.roads = []
        self.sinks_dict = dict()
        self.sinks_list = []
        self.nodes = []
        self.dgraph = {}
        self.intersection_nodes = []
        self.sink_nodes = []
        self.car_number = 0

    # ================ Setup the network ====================
    def addIntersection(self, name, num_lane):
        assert self.is_compiled == False, "The miniVnet has been compiled"

        new_intersection = Intersection(name, num_lane)
        self.intersections[name] = new_intersection
        return new_intersection

    def addSink(self, name, num_lane):
        assert self.is_compiled == False, "The miniVnet has been compiled"

        new_sink = Sink(name, num_lane)
        self.sinks_list.append(new_sink)
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
        intersections = [[self.addIntersection("%3.3o"%(i) + '_' + "%3.3o"%(j), num_lane) for j in range(1,N+1)] for i in
                         range(1,N+1)]

        for i in range(N):
            for j in range(N):
                intersections[i][j].coordinate = (i,j)

        sinks = [[self.addSink(None, num_lane) for j in range(N)] for i in range(4)]

        # Connect intersections
        for idx_x in range(N):
            for idx_y in range(1, N):
                component_1 = intersections[idx_x][idx_y - 1]
                component_2 = intersections[idx_x][idx_y]
                self.connect(component_1, 2, component_2, 0)

        for idx_y in range(N):
            for idx_x in range(1, N):
                component_1 = intersections[idx_x - 1][idx_y]
                component_2 = intersections[idx_x][idx_y]
                self.connect(component_1, 1, component_2, 3)

        # Connect sinks
        for i_idx in range(N):
            target_sink = sinks[0][i_idx]
            target_intersection = intersections[0][i_idx]
            self.connect(target_sink, 0, target_intersection, 3)
            target_sink.set_name("000" + '_' + "%3.3o"%(i_idx+1))
            self.sinks_dict["000" + '_' + "%3.3o"%(i_idx+1)] = target_sink

            target_sink = sinks[1][i_idx]
            target_intersection = intersections[i_idx][N - 1]
            self.connect(target_sink, 0, target_intersection, 2)
            target_sink.set_name("%3.3o"%(i_idx+1) + '_' + "%3.3o"%(N+1))
            self.sinks_dict["%3.3o"%(i_idx+1) + '_' + "%3.3o"%(N+1)] = target_sink

            target_sink = sinks[2][i_idx]
            target_intersection = intersections[N - 1][N - 1 - i_idx]
            self.connect(target_sink, 0, target_intersection, 1)
            target_sink.set_name("%3.3o"%(N+1) + '_' + "%3.3o"%(N - i_idx))
            self.sinks_dict["%3.3o"%(N+1) + '_' + "%3.3o"%(N - i_idx)] = target_sink

            target_sink = sinks[3][i_idx]
            target_intersection = intersections[N - 1 - i_idx][0]
            self.connect(target_sink, 0, target_intersection, 0)
            target_sink.set_name("%3.3o"%(N - i_idx) + '_' + "000")
            self.sinks_dict["%3.3o"%(N - i_idx) + '_' + "000"] = target_sink

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

    def choose_car(self, cars):
        # Choose Cars
        # TODO: dummy function
        chosen_cars = [cars[0]]

        for car in chosen_cars:
            # Remove car from the databases
            for link_id, car_record in car.recorded_in_database.items():
                link = car_record["link"]
                time_cars = car_record["time_car"]
                for time_idx, copied_car in time_cars:
                    link.car_data_base[time_idx].remove(copied_car)
            # Clear paths
            car.path_node = []
            car.path_link = []

        return chosen_cars


    # =========== Use the DataStructure to route ====================== #
    # Important: Not thread safe!! only allow one car routing at a time (because data structure)
    def dijkstra(self, car, src_node, dst_node, time_bias):
        # time_bias is the time bias for those on-the-fly cars, whose is not yet at the source node

        # (initialization)  set every node to be inf. Except the start node= 0
        # Initual Dijkstra graph (This change to enforce "Read-only" for thread safe)
        nodes_arrival_time = dict() # (node_id, Arrival time)
        nodes_from_link = dict() # (node_id, From which link)
        nodes_is_visit = dict() # (node_id, is_visited)
        links_delay = dict() # (link_id, delay)
        links_lane = dict() # (link_id, delay)
        links_delay_record = dict()  # (link_id, {car.id, delay}) Record the new delays of all cars in same batch
        # TODO: see if we want to update AT of all cars

        for intersection in self.intersections.values():
            intersection.initial_for_dijkstra(nodes_arrival_time, nodes_from_link, nodes_is_visit)
        for sink in self.sinks_list:
            sink.initial_for_dijkstra(nodes_arrival_time, nodes_from_link, nodes_is_visit)

        nodes_arrival_time[src_node.id] = (0 + time_bias)

        unvisited_queue = [(nodes_arrival_time[src_node.id], src_node)]

        while len(unvisited_queue) > 0:
            current_arrival_time, current_node = heapq.heappop(unvisited_queue)


            # Only process the first time we pop the node from the queue
            if nodes_is_visit[current_node.id] == True:
                continue
            else:
                nodes_is_visit[current_node.id] = True

            # Update link cost by Roadrunner
            # update when the car is on the road, store the result within the intersection
            # Be aware of the time stemp!!!
            if len(current_node.out_links) > 0:
                turning, out_link = current_node.out_links[0]
                next_node = out_link.out_node

                next_intersection = next_node.get_connect_to_intersection()
                if next_intersection != None:
                    # TODO: build a "new car"
                    new_car = Car()
                    current_arrival_time_idx = int(math.floor(current_arrival_time))
                    new_car.position = out_link.length - ((current_arrival_time - current_arrival_time_idx) * global_val.MAX_SPEED)


                    next_intersection.update_cost_with_manager(current_arrival_time_idx, next_node, new_car, links_delay, links_lane, links_delay_record)


                    '''
                    # TODO: Dummy function
                    for link in next_intersection.links:
                        links_delay[link.id] = 1
                    '''


            # visiting neighbors
            for turning, out_link in current_node.out_links:
                neighbor_node = out_link.out_node

                # Check the value of the neighbors in Dijkstra
                # Only intersection links have delay
                if out_link.is_in_intersection == True:
                    lane_to_lookup = links_lane[out_link.id] % global_val.LANE_NUM_PER_DIRECTION
                    arriving_neighbor_time = current_arrival_time +  out_link.traveling_time + links_delay[out_link.id] + out_link.lane_to_in_inter_time[lane_to_lookup]
                else:
                    arriving_neighbor_time = current_arrival_time +  out_link.traveling_time

                if nodes_arrival_time[neighbor_node.id] > arriving_neighbor_time:
                    nodes_arrival_time[neighbor_node.id] = arriving_neighbor_time
                    nodes_from_link[neighbor_node.id] = out_link
                    heapq.heappush(unvisited_queue, (arriving_neighbor_time, neighbor_node))

            # Early termination when reaching destination
            if current_node.id == dst_node.id:
                break

        '''
        for intersection in self.intersections:
            intersection.print_node_arrival_time()
        print("============================")
        for sink in self.sinks:
            sink.print_node_arrival_time()
        #'''

        car.traveling_time = nodes_arrival_time[dst_node.id]

        # Trace back from destination to source
        tracing_node = dst_node
        car.path_node.insert(0, (nodes_arrival_time[tracing_node.id], tracing_node))
        tracing_link = nodes_from_link[tracing_node.id]

        while tracing_link != None:
            tracing_node = tracing_link.in_node
            car.path_node.insert(0, (nodes_arrival_time[tracing_node.id], tracing_node))

            if tracing_link.id in links_delay:
                car.path_link.insert(0, (links_delay[tracing_link.id], links_lane[tracing_link.id], tracing_link))
            else:
                car.path_link.insert(0, (0, 0, tracing_link))

            tracing_link = tracing_link = nodes_from_link[tracing_node.id]

        assert car.path_link > 0, "The car cannot find the route"
        # TODO: if rerouting: check if it increase global cost
        #       if not: the route must be taken

        '''
        for arrival_time, node in car.path_node:
            print(node, arrival_time)

        for delay, lane, link in car.path_link:
            print(link, link.id, delay, lane)
        '''

    # =========== Update the cost with given path ====================== #
    def update_map(self, car):
        # the time_bias is included during routing

        # path: in_node -----> (link) -----> out_node -----> (next_link)
        for link_idx in range(len(car.path_link)-1):
            _, _, link = car.path_link[link_idx]
            delay, lane, next_link = car.path_link[link_idx+1]
            enter_link_time, in_node = car.path_node[link_idx]
            _, out_node = car.path_node[link_idx+1]

            # Only store the car info in the "road" (connected to the intersection)
            # next_link: links inside the intersection
            if out_node.get_connect_to_intersection() != None:
                # Record which database is added with the car
                car.recorded_in_database[link.id] = {"link":link, "time_car":[]}

                # Get the turns from the node
                turn = out_node.get_turn_from_link(next_link)

                # compute the position on the link
                enter_link_time_idx = int(math.floor(enter_link_time))
                init_position = link.length - ((enter_link_time - enter_link_time_idx) * global_val.MAX_SPEED)

                # compute the traveling time
                actual_travel_time = link.traveling_time + delay

                # expend the list size if not enough
                if len(link.car_data_base) - 1 < enter_link_time_idx:
                    for _ in range(enter_link_time_idx-(len(link.car_data_base) - 1)):
                        link.car_data_base.append([])


                # 1. add the car into the database "unscheduled"
                unscheduled_copy_car = Car()
                unscheduled_copy_car.id = car.id
                unscheduled_copy_car.length = car.length
                unscheduled_copy_car.position = init_position
                unscheduled_copy_car.turning = turn
                unscheduled_copy_car.speed_in_intersection = self.get_speed_in_intersection(turn)
                unscheduled_copy_car.lane = lane
                unscheduled_copy_car.is_scheduled = False
                link.car_data_base[enter_link_time_idx].append(unscheduled_copy_car)
                car.recorded_in_database[link.id]["time_car"].append((enter_link_time_idx,unscheduled_copy_car))


                # 2. add the future car into the database "scheduled"
                current_time_step = 1
                remaining_actual_travel_time = actual_travel_time - current_time_step
                while remaining_actual_travel_time >= 0:
                    current_time_idx = int(math.floor(enter_link_time + current_time_step))
                    if len(link.car_data_base) - 1 < current_time_idx:
                        link.car_data_base.append([])

                    scheduled_copy_car = Car()
                    scheduled_copy_car.id = car.id
                    scheduled_copy_car.length = car.length
                    scheduled_copy_car.arriving_time = remaining_actual_travel_time
                    scheduled_copy_car.turning = turn
                    scheduled_copy_car.speed_in_intersection = self.get_speed_in_intersection(turn)
                    scheduled_copy_car.lane = lane
                    scheduled_copy_car.is_scheduled = True
                    link.car_data_base[current_time_idx].append(scheduled_copy_car)
                    car.recorded_in_database[link.id]["time_car"].append((current_time_idx,scheduled_copy_car))

                    current_time_step = current_time_step+1
                    remaining_actual_travel_time = actual_travel_time - current_time_step


    def get_speed_in_intersection(self, turn):
        turn_speed = 0
        if turn == global_val.STRAIGHT_TURN:
            turn_speed = global_val.MAX_SPEED
        else:
            turn_speed = global_val.TURN_SPEED

        return turn_speed

    def get_car_time_space_list(self, cars):
        all_car_data = {}
        for car in cars:
            car_route = []

            for node_idx in range(len(car.path_node)-1):
                _, in_intersection_node = car.path_node[node_idx]
                intersection = in_intersection_node.get_connect_to_intersection()
                if intersection != None:
                    time, _ = car.path_node[node_idx+1]
                    coordinate = numpy.array([intersection.coordinate[0], intersection.coordinate[1], int(math.floor(time))])
                    car_route.append(coordinate)

            all_car_data[car.id] = numpy.array(car_route)

        io.savemat('test.mat',all_car_data)
