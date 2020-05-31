# Create a network as following:
#         s1  s2
#
#   s8    i1  i2    s3
#   s7    i3  i4    s4
#
#         s6  s5
# Create a network as following:
#         s1  s2  s3
#
#   s12   i1  i2  i3   s4
#   s11   i4  i5  i6   s5
#   s10   i7  i8  i9   s6
#         
#         s9  s8  s7
#
# Index of the initersection:
#       1
#   0   I   2
#       3
#

import time
from miniVnet import MiniVnet
import random
import numpy as np

from basic_graph import Car

random.seed(1)
np.random.seed(1)

def testDijkstra(round_number, car_number):
    tic = time.process_time()

    start = []
    end = []
    for i in range(car_number):
        a = random.sample(range(0,6),1)[0]
        start.append(a)
        end.append(7)
    #print(my_net.sinks[0].out_nodes[0].out_links)
    
    carlist = []
    testpath = []
    for i in range(car_number):
        car = Car()
        car.id = 'Car'+str(i)
        print('-----routing for',car.id,'-----')
        src_node = my_net.sinks[start[i]].out_nodes
        dst_node = my_net.sinks[end[i]].in_nodes
        path = my_net.dijkstra(car, src_node, dst_node, 1)
        if i == 0:
            testpath.append(path)
        carlist.append(car)
        #for link in car.path_link:
        #    print(link.id)
        my_net.dummy_update_map(car)
    ttpath = []
    for k in range(round_number):
        print('======= round ', k, '=============')
        for i in range(car_number):
            car = carlist[i]
            print('------reroute for',car.id,'--------')
            my_net.dummy_reset_map(car)
            src_node = my_net.sinks[start[i]].out_nodes
            dst_node = my_net.sinks[end[i]].in_nodes
            car.path_time = []
            car.path_node = []
            car.path_link = []
            path = my_net.dijkstra(car, src_node, dst_node, 1)
            if i == 0:
                ttpath.append(path)
            my_net.dummy_update_map(car)
    toc = time.process_time()

    print('time',toc-tic)
    print('originial:',testpath)
    print('reroute:',ttpath)



if __name__ == '__main__':
    my_net = MiniVnet()

    i1 = my_net.addIntersection('I1', 3)
    i2 = my_net.addIntersection('I2', 3)
    i3 = my_net.addIntersection('I3', 3)
    i4 = my_net.addIntersection('I4', 3)
    i5 = my_net.addIntersection('I5', 3)
    i6 = my_net.addIntersection('I6', 3)
    i7 = my_net.addIntersection('I7', 3)
    i8 = my_net.addIntersection('I8', 3)
    i9 = my_net.addIntersection('I9', 3)


    s1 = my_net.addSink('S1', 3)
    s2 = my_net.addSink('S2', 3)
    s3 = my_net.addSink('S3', 3)
    s4 = my_net.addSink('S4', 3)
    s5 = my_net.addSink('S5', 3)
    s6 = my_net.addSink('S6', 3)
    s7 = my_net.addSink('S7', 3)
    s8 = my_net.addSink('S8', 3)
    s9 = my_net.addSink('S9', 3)
    s10 = my_net.addSink('S10', 3)
    s11 = my_net.addSink('S11', 3)
    s12 = my_net.addSink('S12', 3)

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
    #testDijkstra(3,20)
    #testDijkstra(4,36)
    #testDijkstra(5,72)
    #testDijkstra(6,80)
    #testDijkstra(7,80)
    #testDijkstra(8,85)
    testDijkstra(9,85)


    



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
