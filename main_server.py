# Create a network as following:
#         s1  s2
#
#   s8    i1  i2    s3
#   s7    i3  i4    s4
#
#         s6  s5
#
# Index of the initersection:
#       1
#   0   I   2
#       3
#


from miniVnet import MiniVnet
import random
import numpy as np

from basic_graph import Car
import time
import threading
import socket
import sys
import global_val
import csv

import traceback
import multiprocessing
from multiprocessing import Manager, Value
import logging

random.seed(0)
np.random.seed(0)
sys.setrecursionlimit(50000)
update_lock = threading.Lock()

def worker(my_net, cars, return_dict, local_lock):
    # My_net is read-only

    logging.info("in worker  1")
    for car in cars:
        #print(car.id)
        my_net.dijkstra(car, car.src_node, car.dst_node, car.time_offset)

        #'''
        my_net.update_map(car)

    try:
        logging.info("in worker  2")
        local_lock.acquire()
        logging.info("in worker  3")
        for car in cars:
            try:
                return_dict[car.id] = car
            finally:
                pass
        logging.info("in worker  4")
    except Exception as e:
        traceback.print_exc()


    local_lock.release()
    logging.info("in worker  5")



def handle_routing(my_net, all_cars_dict, handle_car_dict, new_car, thread_num):

    new_car_list = list(new_car.values())

    route_car_id_dict = dict()
    repeat_time_measure = 0

    manager = multiprocessing.Manager()
    lock = multiprocessing.Lock()
    logging.basicConfig(level=logging.INFO)

    # round robbin x R times
    iteration_num = int(sys.argv[2])
    for _ in range(iteration_num):
        total_cost = 0
        path_diff_count = 0

        choose_car_mode = int(sys.argv[1])
        cars_for_threads, chosen_cars_list = my_net.choose_car(all_cars_dict, handle_car_dict, new_car_list, choose_car_mode)

        # Record which cars are been routed
        for car in chosen_cars_list:
            if car.id not in route_car_id_dict:
                route_car_id_dict[car.id] = car


        is_repeat = True
        while is_repeat: # in case multi-precess fail
            threads = []
            TiStamp1 = time.time()
            is_repeat = False

            try:
                #return_dict = [manager.dict() for idx in range(thread_num)]
                return_dict = manager.dict()
                print("=== 0")
                for thread_idx in range(thread_num):
                    if thread_idx >= len(cars_for_threads):
                        break

                    '''
                    return_dict = dict()
                    t = threading.Thread(target=worker, args=(my_net, cars_for_threads[thread_idx],route_car_id_dict))
                    #'''
                    t = multiprocessing.Process(target=worker, args=(my_net, cars_for_threads[thread_idx],return_dict,lock,))
                    threads.append(t)

                    t.start()

                print("=== 1")
                for thread in threads:
                    thread.join()
                print("=== 2")


        	    #'''
                to_update_car = []
                print(len(return_dict))
                for car in return_dict.values():
                    to_update_car.append(car)
                    #saved_path = car.path_node

                    #total_cost += car.traveling_time
                    #if saved_path != car.path_node and saved_path != []:
                    #    path_diff_count += 1
                print("=== 3")
            except Exception as e:
                traceback.print_exc()
                manager = multiprocessing.Manager()
                lock = multiprocessing.Lock()
                is_repeat = True

            try:
                for process in threads:
                    process.terminate()
                del threads[:]
                #del manager
            except Exception as e:
                traceback.print_exc()
                pass

            TiStamp2 = time.time()

            if is_repeat:
                # Skip update and repeat the routing
                repeat_time_measure += TiStamp2-TiStamp1
                continue
            for car in to_update_car:
                my_net.update_map(car)
                route_car_id_dict[car.id] = car
	#'''

        #print(path_diff_count)
        #print("=============", total_cost/car_num)


        #my_net.get_car_time_space_list(car_list);

    return route_car_id_dict, repeat_time_measure

def SUMO_Handler(sock):

    hello_msg = sock.recv(1024).split(":")
    print(hello_msg)
    grid_size = int(hello_msg[1])
    scheduling_period = int(hello_msg[3])
    routing_period = int(hello_msg[5])
    sock.sendall("Got it ;@")


    thread_num = int(sys.argv[3])
    choose_top_N = int(sys.argv[4])

    # Create network
    my_net = MiniVnet(scheduling_period, routing_period, thread_num, choose_top_N)
    my_net.createGridNetwork(grid_size, 1)

    # Car list
    all_cars_dict = dict()   #(car_id, Car)  Recorded all cars
    all_cars_in_time = dict()

    # Computation time
    compute_time_list = []

    while True:
        handle_cars_dict = dict()   #(car_id, Car)  Cars from SUMO
        new_car_dict = dict()   # To give thses care higher priorities

        # Get data from SUMO
        data = ""
        get_data = ""
        while len(data) == 0 or data[-1] != "@":
            get_data = sock.recv(8192)
            data += get_data
            if get_data == "":
                break
        if data == "END@" or get_data == "":
            break

        TiStamp1 = time.time()

        # Parse data
        data = data[0:-2]   # Remove ";@"
        cars_str_list = data.split(";")
        if len(data) == 0:
            cars_str_list = []  # Clear the element if it is empty
        #print("rcv: ", data)


        for car_str in cars_str_list:
            car_data = car_str.split(",")
            car_id = car_data[0]
            car_status = car_data[1]

            if car_status == "Exit":
                #print(car_id, all_cars_dict[car_id].traveling_time+2*(200/global_val.MAX_SPEED)-all_cars_in_time[car_id])
                del all_cars_dict[car_id]
                del all_cars_in_time[car_id]
            else:
                if car_status == "NEW":
                    car = Car()
                    car.id = car_id
                    car.lentgh = float(car_data[2])

                    dst_sink_id = int(car_data[3])
                    car.dst_node = my_net.sinks_list[dst_sink_id].in_nodes

                    all_cars_dict[car_id] = car
                    new_car_dict[car_id] = car

                    all_cars_in_time[car_id] = float(car_data[4])

                # Parse source node
                car = all_cars_dict[car_id]
                handle_cars_dict[car_id] = car

                car.src_node = None

                time_offset = float(car_data[4])
                car.time_offset = time_offset
                intersection_id = car_data[5]
                inter_id_list = intersection_id.split("_")
                inter_id_x = int(inter_id_list[0])
                inter_id_y = int(inter_id_list[1])

                car.src_node_str = intersection_id

                if inter_id_x == 0 or inter_id_y == 0 or inter_id_x == grid_size+1 or inter_id_y == grid_size+1:
                    # Sinc
                    car.src_node = my_net.sinks_dict[intersection_id].out_nodes
                    car.is_src_sink = True
                else:
                    # Intersection
                    intersection_direction = int(car_data[6])
                    car.src_node = my_net.intersections[intersection_id].out_nodes[intersection_direction]
                    car.is_src_sink = False

        # Handle the route
        route_car_id_dict, repeat_time_measure = handle_routing(my_net, all_cars_dict, handle_cars_dict, new_car_dict, thread_num)


        # Construct message
        server_send_str = ""
        for car_id, car in route_car_id_dict.items():
            server_send_str += car_id + ","

            if car.is_src_sink:
                server_send_str += car.src_node_str + ":S/"

            # path: in_node -----> (link) -----> out_node -----> (next_link)
            for link_idx in range(len(car.path_link)-1):
                _, _, link = car.path_link[link_idx]
                delay, lane, next_link = car.path_link[link_idx+1]
                enter_link_time, in_node = car.path_node[link_idx]
                _, out_node = car.path_node[link_idx+1]

                intersection = out_node.get_connect_to_intersection()
                if intersection != None:
                    server_send_str += intersection.name + ":"
                    turning = out_node.link_to_turn[next_link]

                    if turning == global_val.LEFT_TURN:
                        server_send_str += "L/"
                    elif turning == global_val.STRAIGHT_TURN:
                        server_send_str += "S/"
                    elif turning == global_val.RIGHT_TURN:
                        server_send_str += "R/"
                    else:
                        # For debug
                        print("ERROR: No turning")

            server_send_str += ";"

        server_send_str += "@"


        computation_time = time.time() - TiStamp1 - repeat_time_measure
        print(computation_time, "sec")
        if repeat_time_measure == 0:
            # Not recording the computation time caused by system multiprocessing issue
            compute_time_list.append(computation_time)
        else:
            print("Skip time record")


        #print("snd: ", server_send_str)
        sock.sendall(server_send_str)


        # My net take a time step
        my_net.take_a_step(all_cars_dict)

        sys.stdout.flush()

    compute_time_list = compute_time_list
    average_computation_time = sum(compute_time_list)/len(compute_time_list)
    max_computation_time = max(compute_time_list)


    with open('../result/computation_time.csv', 'a') as csvfile:
        file_writer = csv.writer(csvfile, lineterminator='\n')
        to_write_list = [sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7]]
        to_write_list.append(average_computation_time)
        to_write_list.append(max_computation_time)
        file_writer.writerow(to_write_list)



if __name__ == '__main__':
    print("Usage: python code.py <choose_car_algorithm> <iteration_num> <thread_num> <Top N number>")
    print("--------------------- <arrival_rate> <rand_seed> <grid_size>")
    sys.argv[4]
    sys.argv[7]

    #HOST, PORT = "128.238.147.124", 9999
    HOST, PORT = "localhost", 9909
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.bind((HOST, PORT))
    print("start server")

    try:
        server_sock.listen(5)
        sock, addr = server_sock.accept()
        print("Got a connection from: ", addr)
        SUMO_Handler(sock)
    except Exception as e:
        traceback.print_exc()
