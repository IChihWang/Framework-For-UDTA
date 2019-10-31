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


from miniVnet import MINIVNET


if __name__ == '__main__':
    my_net = MINIVNET()
    
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
    
    my_net.connect(i1, 0, s8, 0)
    my_net.connect(i1, 1, s1, 0)
    my_net.connect(i1, 2, i2, 0)
    my_net.connect(i1, 3, i3, 1)
    
    my_net.connect(i2, 1, s2, 0)
    my_net.connect(i2, 2, s3, 0)
    my_net.connect(i2, 3, i4, 1)
    
    my_net.connect(i3, 0, s7, 0)
    my_net.connect(i3, 2, i4, 0)
    my_net.connect(i3, 3, s6, 0)
    
    my_net.connect(i4, 2, s4, 0)
    my_net.connect(i4, 3, s5, 0)
    
    my_net.compile()



#a = {i:int(np.random.randint(10, size=1)) for i in range(10)}

#my_net.debug()
graph = my_net.dijkstraGraph()
#print(graph)
p1 = my_net.newdijkstra(0,2)
#p2 = my_net.dijkstra(graph,0,2)
#p3 = my_net.dijkstra(graph,0,3)
#p4 = my_net.dijkstra(graph,2,1)
#p5 = my_net.dijkstra(graph,3,2)

#my_net.countCar(graph,p1)
#print(graph['SINK S2'])