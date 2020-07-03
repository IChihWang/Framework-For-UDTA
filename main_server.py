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


random.seed(0)
np.random.seed(0)


def worker(my_net, cars):
    # My_net is read-only
    for car in cars:
        my_net.dijkstra(car, car.src_node, car.dst_node, car.time_offset)

def handle_routing(my_net, car_dict, new_car_dict):

    car_list = list(new_car_dict.values())
    #car_list = list(car_dict.values())
    route_car_id_dict = dict()

    thread_num = 4
    car_num_per_thread = len(car_list)//thread_num+1
    TiStamp1 = time.time()


    # round robbin x 5 times
    for _ in range(1):
        total_cost = 0
        path_diff_count = 0

        # Record which cars are been routed
        for car in car_list:
            if car.id not in route_car_id_dict:
                route_car_id_dict[car.id] = car

        # TODO: Dummy for debug
        for car in car_list:
            my_net.choose_car([car])

        threads = []
        for car_idx in range(0, len(car_list), car_num_per_thread):
            if car_idx+car_num_per_thread > len(car_list):
                cars = car_list[car_idx:len(car_list)]
            else:
                cars = car_list[car_idx:car_idx+car_num_per_thread]

            # clear car from the database

            #car = cars[0]


            t = threading.Thread(target=worker, args=(my_net, cars,))
            threads.append(t)
            t.start()

        for thread in threads:
            thread.join()

        for car in car_list:

            my_net.update_map(car)

            saved_path = car.path_node
            total_cost += car.traveling_time
            if saved_path != car.path_node and saved_path != []:
                #print(car.id, car.path_node)
                path_diff_count += 1

            sys.stdout.flush()
        #print(path_diff_count)
        #print("=============", total_cost/car_num)


    print(time.time() - TiStamp1, "sec")
    my_net.get_car_time_space_list(car_list);

    return route_car_id_dict

def SUMO_Handler(sock):

    hello_msg = sock.recv(1024).strip(" ")
    grid_size = int(hello_msg[0])
    sock.sendall("Got it ;@")

    # Create network
    my_net = MiniVnet()
    my_net.createGridNetwork(grid_size, 1)

    # Car list
    cars_dict = dict()   #(car_id, Car)

    while True:

        # Get data from SUMO
        data = ""
        while len(data) == 0 or data[-1] != "@":
            data += sock.recv(8192)
        # Parse data
        data = data[0:-2]   # Remove ";@"
        cars_str_list = data.split(";")
        print("rcv: ", data)

        new_car_dict = dict()   # To give thses care higher priorities

        for car_str in cars_str_list:
            car_data = car_str.split(",")
            car_id = car_data[0]
            car_status = car_data[1]

            if car_status == "Exit":
                del cars_dict[car_id]
            else:
                if car_status == "NEW":
                    car = Car()
                    car.id = car_id
                    car.lentgh = float(car_data[2])

                    dst_sink_id = int(car_data[3])
                    car.dst_node = my_net.sinks_list[dst_sink_id].in_nodes

                    cars_dict[car_id] = car
                    new_car_dict[car_id] = car

                time_offset = float(car_data[4])
                car.time_offset = time_offset

                # Parse source node
                car = cars_dict[car_id]
                car.src_node = None
                intersection_id = car_data[5]
                inter_id_list = intersection_id.split("_")
                inter_id_x = int(inter_id_list[0])
                inter_id_y = int(inter_id_list[1])

                if inter_id_x == 0 or inter_id_y == 0 or inter_id_x == grid_size+1 or inter_id_y == grid_size+1:
                    # Sinc
                    car.src_node = my_net.sinks_dict[intersection_id].out_nodes
                else:
                    # Intersection
                    intersection_direction = int(car_data[6])
                    car.src_node = my_net.intersections[intersection_id].out_nodes[intersection_direction]

        # Handle the route
        route_car_id_dict = handle_routing(my_net, cars_dict, new_car_dict)


        # Construct message
        server_send_str = ""
        for car_id, car in route_car_id_dict.items():
            server_send_str += car_id + ","

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


        print("snd: ", server_send_str)
        sock.sendall(server_send_str)

        sys.stdout.flush()


if __name__ == '__main__':
    HOST, PORT = "128.238.147.124", 9999
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.bind((HOST, PORT))
    print("start server")

    try:
        server_sock.listen(5)
        sock, addr = server_sock.accept()
        print("Got a connection from: ", addr)
        SUMO_Handler(sock)
    except:
        pass
