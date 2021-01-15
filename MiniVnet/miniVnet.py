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
    def __init__(self, scheduling_period, routing_period, thread_num, choose_top_N):
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
        self.scheduling_period = scheduling_period  # Time for sheduling
        self.routing_period = routing_period    # How many scheduling periods per routing

        self.route_database_num = choose_top_N    # Top 10 congested
        self.route_database_list = []   # [(time_idx,database),]
        self.thread_num = thread_num
        self.min_database_per_thread = math.ceil(float(self.route_database_num)/self.thread_num)

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
        intersections = [[self.addIntersection("00%i"%(i) + '_' + "00%i"%(j), num_lane) for j in range(1,N+1)] for i in
                         range(1,N+1)]
        '''
        for idx in range(0, N):
            for jdx in range(0, N):
                if idx <= N-2:
                    intersections[idx][jdx].connect_intersection(1, intersections[idx+1][jdx], 3)

                if jdx <= N-2:
                    intersections[idx][jdx].connect_intersection(2, intersections[idx][jdx+1], 0)
        '''

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
            target_sink.set_name("000" + '_' + "00%i"%(i_idx+1))
            self.sinks_dict["000" + '_' + "00%i"%(i_idx+1)] = target_sink

            target_sink = sinks[1][i_idx]
            target_intersection = intersections[i_idx][N - 1]
            self.connect(target_sink, 0, target_intersection, 2)
            target_sink.set_name("00%i"%(i_idx+1) + '_' + "00%i"%(N+1))
            self.sinks_dict["00%i"%(i_idx+1) + '_' + "00%i"%(N+1)] = target_sink

            target_sink = sinks[2][i_idx]
            target_intersection = intersections[N - 1][N - 1 - i_idx]
            self.connect(target_sink, 0, target_intersection, 1)
            target_sink.set_name("00%i"%(N+1) + '_' + "00%i"%(N - i_idx))
            self.sinks_dict["00%i"%(N+1) + '_' + "00%i"%(N - i_idx)] = target_sink

            target_sink = sinks[3][i_idx]
            target_intersection = intersections[N - 1 - i_idx][0]
            self.connect(target_sink, 0, target_intersection, 0)
            target_sink.set_name("00%i"%(N - i_idx) + '_' + "000")
            self.sinks_dict["00%i"%(N - i_idx) + '_' + "000"] = target_sink

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

    def choose_car(self, all_cars_dict, handle_car_dict, new_car, choose_car_mode):
        # Choose Cars
        # TODO: dummy function


        '''
        time_label_idx, inter_, compare_database = self.route_database_list[database_idx]
        '''
        cars_for_threads = []   # list of list of cars
        chosen_cars = []

        if choose_car_mode == 0:
            if self.thread_num == 1:
                # Top N intersection
                for _, _, car_id_list in self.route_database_list:
                    for car_id in car_id_list:
                        car = all_cars_dict[car_id]
                        if car not in chosen_cars and car in handle_car_dict.values():
                            chosen_cars.append(car)

                for car in new_car:
                    if car not in chosen_cars:
                        chosen_cars += new_car
                cars_for_threads.append(chosen_cars)
            else:
                top_N_list = []
                # Top N intersection
                for _, _, car_id_list in self.route_database_list:
                    top_N_list.append(car_id_list)

                bfs_tree_idx = -1   # Start from zero. Tricky part for while
                bfs_tree_list = []     # M threads- M list- with k of the top_N_list


                while len(top_N_list) > 0:
                    bfs_tree_idx += 1
                    bfs_tree_list.append([])
                    queue = [top_N_list[0]]
                    visited = [top_N_list[0]]
                    bfs_tree_list[bfs_tree_idx].append(top_N_list[0])

                    # BFS
                    while len(queue) > 0:
                        current_database = queue.pop(0)
                        for database_list in top_N_list:
                            if database_list not in visited:
                                if len(set(current_database) & set(database_list)) > 0:
                                    bfs_tree_list[bfs_tree_idx].append(database_list)
                                    visited.append(database_list)
                                    queue.append(database_list)

                    for database_list in bfs_tree_list[bfs_tree_idx]:
                        top_N_list.remove(database_list)
                while len(bfs_tree_list) > self.thread_num:
                    # merge the list
                    for database_lists in bfs_tree_list:
                        if len(database_lists) < self.min_database_per_thread:

                            min_bfs_tree = min(bfs_tree_list, key=lambda x: len(x))
                            bfs_tree_list.remove(min_bfs_tree)
                            database_lists += min_bfs_tree
                            break

                # convert the database_list to cars
                for thread_idx in range(len(bfs_tree_list)):
                    cars_for_threads.append([])
                    for car_id_list in bfs_tree_list[thread_idx]:
                        for car_id in car_id_list:
                            car = all_cars_dict[car_id]
                            if car not in chosen_cars and car in handle_car_dict.values():
                                chosen_cars.append(car)
                                cars_for_threads[thread_idx].append(car)
                for _ in range(self.thread_num-len(cars_for_threads)):
                    cars_for_threads.append([])

                for car in new_car:
                    if car not in chosen_cars:
                        # Insert the car to the thread with the minimal numbers of cars
                        min_ars_for_thread = min(cars_for_threads, key=lambda x: len(x))
                        min_ars_for_thread.append(car)

                chosen_cars += new_car
            del new_car[:]
        elif choose_car_mode == 1:
            if self.thread_num == 1:
                # Top N intersection
                for _, _, car_id_list in self.route_database_list:
                    for car_id in car_id_list:
                        car = all_cars_dict[car_id]
                        if car not in chosen_cars and car in handle_car_dict.values():
                            chosen_cars.append(car)

                for car in new_car:
                    if car not in chosen_cars:
                        chosen_cars += new_car
                cars_for_threads.append(chosen_cars)
            else:
                cars_for_threads.append([])
                cars_for_threads_idx = 0
                route_database_count = 0
                # Top N intersection
                for _, _, car_id_list in self.route_database_list:
                    if route_database_count > self.min_database_per_thread:
                        route_database_count = 0
                        cars_for_threads.append([])
                        cars_for_threads_idx += 1

                    for car_id in car_id_list:
                        car = all_cars_dict[car_id]
                        if car not in chosen_cars and car in handle_car_dict.values():
                            chosen_cars.append(car)
                            cars_for_threads[cars_for_threads_idx].append(car)

                    route_database_count += 1


                for _ in range(self.thread_num-len(cars_for_threads)):
                    cars_for_threads.append([])

                for car in new_car:
                    if car not in chosen_cars:
                        chosen_cars.append(car)
                        # Insert the car to the thread with the minimal numbers of cars
                        min_ars_for_thread = min(cars_for_threads, key=lambda x: len(x))
                        min_ars_for_thread.append(car)

                #chosen_cars += new_car
            del new_car[:]

        elif choose_car_mode == 2:
            # Only for one thread
            for _ in range(self.thread_num-1):
                cars_for_threads.append([])
            chosen_cars += new_car
            cars_for_threads.append(chosen_cars)
            del new_car[:]
        elif choose_car_mode == 3:
            # Only for one thread
            for _ in range(self.thread_num-1):
                cars_for_threads.append([])
            chosen_cars = handle_car_dict.values()
            cars_for_threads.append(chosen_cars)
        else:
            print("Wrong choose car mode")
            exit()

        # TODO: decide Thread


        for car in chosen_cars:
            # Remove car from the databases
            for link_id, car_record in car.recorded_in_database.items():
                link = car_record["link"]
                time_cars = car_record["time_car"]
                for time_idx, copied_car in time_cars:
                    current_time_idx = time_idx - car.time_offset_counter
                    if current_time_idx >= 0:
                        link.car_data_base[current_time_idx].remove(copied_car)


            # Set the time offset counter to zero
            car.time_offset_counter = 0

            # Clear paths
            car.path_node = []
            car.path_link = []
            car.recorded_in_database = dict()


        return cars_for_threads, chosen_cars


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
                    new_car.id = car.id
                    current_arrival_time_idx = int(current_arrival_time//self.scheduling_period)
                    new_car.position = out_link.length - ((current_arrival_time - current_arrival_time_idx*self.scheduling_period) * global_val.MAX_SPEED)

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

    def add_database_to_sort_database(self, intersection, time_idx):

        # The node is going to be removed, so skip
        if time_idx < self.routing_period:
            return

        # Maintain a list with top "self.route_database_num" congested links
        car_id_list = []
        for in_node in intersection.in_nodes:
            link = in_node.in_links[0]
            if len(link.car_data_base) > time_idx:
                for car in link.car_data_base[time_idx]:
                    if car.position != None:
                        car_id_list.append(car.id)

        car_num_in_database = len(car_id_list)

        # Remove the duplicated one from the route_database_list
        rm_idx = None
        for idx in range(len(self.route_database_list)):
            t_idx, inter, _ = self.route_database_list[idx]
            if (t_idx, inter) == (time_idx, intersection):
                rm_idx = idx
                break
        if rm_idx != None:
            self.route_database_list.pop(rm_idx)

        # Insert the database
        insert_database_idx = 0
        for database_idx in reversed(range(len(self.route_database_list))):
            time_label_idx, inter_, compare_database = self.route_database_list[database_idx]
            if len(compare_database) > car_num_in_database:
                insert_database_idx = database_idx+1
                break
        self.route_database_list.insert(insert_database_idx, (time_idx, intersection, car_id_list))

        # Remove the extra one
        if len(self.route_database_list) > self.route_database_num:
            self.route_database_list.pop()


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
            current_intersection = out_node.get_connect_to_intersection()
            if current_intersection != None:
                # Record which database is added with the car
                car.recorded_in_database[link.id] = {"link":link, "time_car":[]}
                #if car.id == "car_4":
                    #print(car.id, len(car.path_link))
                # Get the turns from the node
                turn = out_node.get_turn_from_link(next_link)

                # compute the position on the link
                enter_link_time_idx = int(enter_link_time//self.scheduling_period)
                init_position = link.length - ((enter_link_time - enter_link_time_idx*self.scheduling_period) * global_val.MAX_SPEED)

                # compute the traveling time
                actual_travel_time = link.traveling_time + delay

                # expend the list size if not enough
                if len(link.car_data_base) - 1 < enter_link_time_idx:
                    for _ in range(0, enter_link_time_idx-(len(link.car_data_base) - 1)):
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

                self.add_database_to_sort_database(current_intersection, enter_link_time_idx)


                # 2. add the future car into the database "scheduled"
                current_time_step = self.scheduling_period
                remaining_actual_travel_time = actual_travel_time - current_time_step
                while remaining_actual_travel_time >= 0:
                    current_time_idx = int((enter_link_time + current_time_step)//self.scheduling_period)
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

                    self.add_database_to_sort_database(current_intersection, current_time_idx)

                    current_time_step += self.scheduling_period
                    remaining_actual_travel_time = actual_travel_time - current_time_step

    # =========== take a time step ====================== #
    def take_a_step(self, all_cars_dict):
        # Handle car
        for car in all_cars_dict.values():
            car.time_offset_counter += self.routing_period

        # Handle link database
        for road in self.roads:
            for link in road.link_groups:
                link.car_data_base = link.car_data_base[self.routing_period:]

        # Handle the congested link list
        delete_database_list = []
        for database_idx in range(len(self.route_database_list)):
            time_label_idx, intersection, car_id_list = self.route_database_list[database_idx]
            time_label_idx -= self.routing_period
            if time_label_idx >= 0:
                self.route_database_list[database_idx] = (time_label_idx, intersection, car_id_list)
            else:
                delete_database_list.append(database_idx)
        for database_idx in reversed(delete_database_list):
            del self.route_database_list[database_idx]



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
