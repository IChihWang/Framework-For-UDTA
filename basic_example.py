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

random.seed(0)
np.random.seed(0)

if __name__ == '__main__':
    my_net = MiniVnet()

    grid_size = 10
    my_net.createGridNetwork(grid_size, 1)

    '''
    i1 = my_net.addIntersection('I1', 3)
    i2 = my_net.addIntersection('I2', 3)
    i3 = my_net.addIntersection('I3', 3)
    i4 = my_net.addIntersection('I4', 3)

    s1 = my_net.addSink('S1', 3)
    s2 = my_net.addSink('S2', 3)
    s3 = my_net.addSink('S3', 3)
    s4 = my_net.addSink('S4', 3)
    s5 = my_net.addSink('S5', 3)
    s6 = my_net.addSink('S6', 3)
    s7 = my_net.addSink('S7', 3)
    s8 = my_net.addSink('S8', 3)

    my_net.connect(i1, 3, s8, 0)
    my_net.connect(i1, 2, s1, 0)
    my_net.connect(i1, 1, i2, 3)
    my_net.connect(i1, 0, i3, 2)

    my_net.connect(i2, 2, s2, 0)
    my_net.connect(i2, 1, s3, 0)
    my_net.connect(i2, 0, i4, 2)

    my_net.connect(i3, 3, s7, 0)
    my_net.connect(i3, 1, i4, 3)
    my_net.connect(i3, 0, s6, 0)

    my_net.connect(i4, 1, s4, 0)
    my_net.connect(i4, 0, s5, 0)

    my_net.compile()
    '''

    '''
    i1.print_details()
    i2.print_details()
    i3.print_details()
    i4.print_details()

    s1.print_details()
    s2.print_details()
    s3.print_details()
    s4.print_details()
    s5.print_details()
    s6.print_details()
    s7.print_details()
    s8.print_details()
    '''

    #print(my_net.sinks[0].out_nodes[0].out_links)


    '''
    car = Car()
    car.id = "hello_car"
    car.lentgh = 5
    src_node = my_net.sinks[0].out_nodes
    dst_node = my_net.sinks[4].in_nodes
    my_net.dijkstra(car, src_node, dst_node, 1)
    my_net.update_map(car)
    print(car.recorded_in_database)

    my_net.choose_car([car])
    '''


    car_list = []
    car_num = 20
    for idx in range(car_num):
        car = Car()
        car.id = "car_" + str(idx)
        car.lentgh = 5

        src_node_idx = random.randrange(0,grid_size*4)
        dst_node_idx = src_node_idx
        while src_node_idx == dst_node_idx:
            dst_node_idx = random.randrange(0,grid_size*4)

        car.src_node = my_net.sinks[src_node_idx].out_nodes
        car.dst_node = my_net.sinks[dst_node_idx].in_nodes
        car_list.append(car)


    TiStamp1 = time.time()
    # round robbin x 5 times
    for _ in range(1):
        total_cost = 0
        path_diff_count = 0
        for car in car_list:
            saved_path = car.path_node

            # clear car from the database

            cars = my_net.choose_car([car])
            car = cars[0]
            my_net.dijkstra(car, car.src_node, car.dst_node, 0)
            my_net.update_map(car)


            total_cost += car.traveling_time
            #'''
            if saved_path != car.path_node and saved_path != []:
                #print(car.id, car.path_node)
                path_diff_count += 1
            #'''

            '''
            if car.id == "car_13":
                print(car.path_node)
            #'''

            print(car.id)
        #print(path_diff_count)
        #print("=============", total_cost/car_num)


    print(time.time() - TiStamp1, "sec")
    my_net.get_car_time_space_list(car_list);




'''

# Baseline. the original path dijkstra
paths1 = []
times1 = []
for _ in range(5):
    a,b = random.sample(range(0,7),2)
    p,t = my_net.dijkstra(a,b)
    paths1.append(p)
    times1.append(t)


# Update the link to see if the balance happened
paths2 = []
times2 = []
for _ in range(5):
    a,b = random.sample(range(0,7),2)
    p,t = my_net.dijkstra(a,b)
    paths2.append(p)
    times2.append(t)
    my_net.updateLinkCost(p,t)

# reroute part#
## redo linkcost
## dijkstra

# Reset the cost link
paths3 = []
times3 = []
for _ in range(5):
    p = paths2[_]
    t = times2[_]
    my_net.resetLinkCost(p,t)
    a,b = random.sample(range(0,7),2)
    p,t = my_net.dijkstra(a,b)
    paths3.append(p)
    times3.append(t)

# Finish the undoLinkCost

my_net.multipleIntCount(paths1,times1)
my_net.multipleIntCount(paths2,times2)
my_net.multipleIntCount(paths3,times3)

'''
