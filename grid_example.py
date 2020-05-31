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
import random
import numpy as np
from miniVnet import MiniVnet

from basic_graph import Car

random.seed(1)
np.random.seed(1)

def testDijkstra(round_number, car_number):
    import time

    tic = time.clock()
    start = []
    end = []
    kk = 0
    testpath = {}
    ttest = {}

    for i in range(car_number):
        a,b = random.sample(range(0,7),2)
        testpath[i] = []
        ttest[i] = []
        start.append(a)
        end.append(b)
    #print(my_net.sinks[0].out_nodes[0].out_links)
    carlist = []
    for i in range(car_number):
        car = Car()
        car.id = 'Car'+str(i)
        #print('-----routing for',car.id,'-----')
        src_node = my_net.sinks[start[i]].out_nodes
        dst_node = my_net.sinks[end[i]].in_nodes
        path = my_net.dijkstra(car, src_node, dst_node, 1)
        testpath[i].append(path)
        carlist.append(car)
        #for link in car.path_link:
        #    print(link.id)
        my_net.dummy_update_map(car)
    for k in range(round_number):
        #print('======= round ', k, '=============')
        for i in range(car_number):
            car = carlist[i]
            #print('------reroute for',car.id,'--------')
            my_net.dummy_reset_map(car)
            src_node = my_net.sinks[start[i]].out_nodes
            dst_node = my_net.sinks[end[i]].in_nodes
            car.path_time = []
            car.path_node = []
            car.path_link = []
            path = my_net.dijkstra(car, src_node, dst_node, 1)
            #print('testpath',testpath[i])
            #print('path',path)
            testpath[i].append(path)
            #if path == testpath[i]:
            #    ttest[i].append(k)
            my_net.dummy_update_map(car)
    
    toc = time.clock()
    for i in range(car_number):
        for elem in testpath[i]:
            if elem not in ttest[i]:
                ttest[i].append(elem)
    kk = 0
    for i in range(car_number):
        if len(ttest[i]) > 1:
            #print(ttest[i])
            kk += 1
    return kk, toc-tic 
            
    # kk = 0
    # for i in range(car_number):
    #     if len(ttest[i]) == 0:
    #         kk += 1
    # print(ttest)
    # return kk
    #for i in range(car_number):
    #    if len(car)
    #return newpath, toc-tic
    #print('originial:',testpath)
    #print('reroute:',ttpath)
    

if __name__ == '__main__':
    my_net = MiniVnet()
    
    my_net.createGridNetwork(4, 4)
    timelist = []
    for i in range(35,52):
        print(i)
        kk,time = testDijkstra(3,i)
        timelist.append(time)
    print(timelist)
    # for r in range(6,10):
    #     for carn in range(80,200):
            
    #         kk,time = testDijkstra(r,carn)
    #         if kk > carn/3*2:
    #             print('round#:', r)
    #             print('car#:',carn)
    #             print('kk#:',kk)
    #         # #if kk > carn - r:
            #    
            #     break
            #     #print(newpath)
        #print(len(newpath))
        # if len(newpath) == r:
        #     print('round:', r)
        #     print('car_number:', carn)
        #     print('time:', time)
        #     break
                