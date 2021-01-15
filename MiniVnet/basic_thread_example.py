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

random.seed(0)
np.random.seed(0)


def worker(my_net, cars):
    for car in cars:
        my_net.dijkstra(car, car.src_node, car.dst_node, 0)

if __name__ == '__main__':
    my_net = MiniVnet()

    grid_size = 10
    my_net.createGridNetwork(grid_size, 1)


    car_list = []
    car_num = 50
    for idx in range(car_num):
        car = Car()
        car.id = "car" + str(idx)
        car.lentgh = 5

        src_node_idx = random.randrange(0,grid_size*4)
        dst_node_idx = src_node_idx
        while src_node_idx == dst_node_idx:
            dst_node_idx = random.randrange(0,grid_size*4)

        car.src_node = my_net.sinks[src_node_idx].out_nodes
        car.dst_node = my_net.sinks[dst_node_idx].in_nodes
        car_list.append(car)

    thread_num = 5
    thread_step = len(car_list)/thread_num
    TiStamp1 = time.time()
    # round robbin x 5 times
    for _ in range(1):
        total_cost = 0
        path_diff_count = 0


        threads = []
        for car_idx in range(0, len(car_list), thread_step):
            cars = car_list[car_idx:car_idx+thread_step]

            # clear car from the database

            #cars = my_net.choose_car([car])
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
            #'''
            if saved_path != car.path_node and saved_path != []:
                #print(car.id, car.path_node)
                path_diff_count += 1
            #'''

            print(car.id)
        #print(path_diff_count)
        #print("=============", total_cost/car_num)


    print(time.time() - TiStamp1, "sec")
    my_net.get_car_time_space_list(car_list);
