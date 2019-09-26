from basicGraph import NODE, LINK

class INTERSECTION:
    def __init__(self, name, num_lane):
        self.name = name
        self.num_lane = num_lane
        self.components = [None for i in range(4)]
        self.direction_nodes = [[[NODE() for k in range(num_lane)] for j in range(2)] for i in range(4)]
        
        # Allocate links to nodes
        for i in range(4):
            for j in range(num_lane):
                for k in range(3):  # left, straight, right
                    source_node = self.direction_nodes[i][0][j]   
                    new_link = LINK()
                    source_node.out_links.append(new_link)
                    new_link.in_node = source_node
                    
                    sink_node = self.direction_nodes[(i+1+k)%4][1][j]
                    sink_node.in_links.append(new_link)
                    new_link.out_node = sink_node
        
        
    def connect(self, index, road):
        assert self.components[index] == None, "Two components are assigned to the same entry of the intersection: " + str(self.name)
        
        assert index < 4, "Index out of range, intersection: " + str(self.name)
        
        self.components[index] = road
        
        road.connect(self, self.direction_nodes[index][1], self.direction_nodes[index][0])
        
    def checkSetting(self):
        component_list = [component for component in self.components if component != None]
        
        assert len(component_list) > 2, "Error: intersection " + str(self.name) + " has too few connections."
    
    def debug(self):
        print (self.name, self.num_lane, self.components)
        
            


class ROAD:
    def __init__(self, num_lane):
        self.num_lane = num_lane
        self.link_groups = [[LINK() for j in range(num_lane)] for i in range(2)]
        self.components = [None for i in range(2)]
        
    def connect(self, component, in_nodes, out_nodes):
        assert self.components[0] == None or self.components[1] == None, "A road is overly assigned"
        
        if self.components[0] == None:
            self.components[0] = component
            for i in range(self.num_lane):
                self.link_groups[0][i].in_node = in_nodes[i]
                self.link_groups[1][i].out_node = out_nodes[i]
        else:
            self.components[1] = component
            for i in range(self.num_lane):
                self.link_groups[1][i].in_node = in_nodes[i]
                self.link_groups[0][i].out_node = out_nodes[i]

#    def checkSetting(self):
#        component_list = [component for component in self.components if component != None]
#        
#      assert len(component_list) == 2, "Error: Road " + str(self.name) + " has too few connections."


class SINK:
    def __init__(self, name, num_lane):
        self.name = name
        self.num_lane = num_lane
        self.node_groups = [[NODE() for i in range(num_lane)] for i in range(2)]
        self.components = None
        
    def connect(self, index, road):
        assert self.components == None, "The sink is already connected, sink: " + str(self.name)
        
        assert index == 0, "Index out of range, sink: " + str(self.name)
        
        self.components = road
                    
        road.connect(self, self.node_groups[0], self.node_groups[1])
        
    def checkSetting(self):
        assert self.components != None, "Error: sink " + str(self.name) + " has too few connections."


class MINIVNET:
    def __init__(self):
        self.is_compiled = False
        self.intersections = []
        self.roads = []
        self.sinks = []
        self.dgraph = {}

    
    
    # ================ Setup the network ====================
    def addIntersection(self, name, num_lane):
        assert self.is_compiled == False, "The miniVnet has been compiled"
    
        new_intersection = INTERSECTION(name, num_lane)
        self.intersections.append(new_intersection)
        return new_intersection
        
        
    def addSink(self, name, num_lane):
        assert self.is_compiled == False, "The miniVnet has been compiled"
    
        new_sink = SINK(name, num_lane)
        self.sinks.append(new_sink)
        return new_sink
        
    def connect(self, component_1, idx_1, component_2, idx_2):
        assert self.is_compiled == False, "The miniVnet has been compiled"
        assert component_1.num_lane == component_2.num_lane, "Lane number mismatch: " + str(component_1.name) + " " + str(component_2.name)
        
        num_lane = component_1.num_lane
    
        new_road = ROAD(num_lane)
        self.roads.append(new_road)
        component_1.connect(idx_1, new_road)
        component_2.connect(idx_2, new_road)
    
    
    def debug(self):
        #for i in range(len(self.roads)):
        #    graph_road = self.roads[i]
            #print(i,graph_road)
        #    linkg = graph_road.link_groups[0]
            #print(i,linkg[0].in_node)
            #for j in range(len(graph_road.link_groups)):
             #   linkg = graph_road[0].link_groups[j]
             #   print(j,linkg)
                #for k in range(len(linkg)):
                    #link = linkg[k]
                    #out_node = link.out_node
                  #  print(,j,k,link.out_node,'\n')
            #print(i,graph_road.link_groups,'\n')

        # for i in range(len(self.sinks)):
        #     sink = self.sinks[i]
        #     self.dgraph[sink] = {}
        #     for j in range(len(sink.node_groups)):
        #         nodes = sink.node_groups[j]
        #         for jj in range(len(nodes)):
        #             print(nodes[jj])
        
        for i in range(len(self.roads)):
            graph_road = self.roads[i]
            linkg = graph_road.link_groups
            for j in range(2):
                for link in linkg[j]:
                    print(link.in_node)
        
    def dijkstraGraph(self):
        # for k in range(len(self.roads)):
        #     graph_road = self.roads[k]
        #     linkg = graph_road.link_groups
        #     for l in range(2):
        #         # Search through every link
        #         for link in linkg[l]:
        #         # In the same Sink
        #             for i in range(len(self.sinks)):
        #                 sink = self.sinks[i]
        #                 self.dgraph[sink] = {}
        #                 for j in range(len(sink.node_groups)):
        #                     snodes = sink.node_groups[j]
        #                     for jj in range(len(snodes)):
        #                         snode = snodes[jj]
        #                         if link.in_node is snode:
        #                             self.dgraph[sink][link.out_node] = link.cost

        #         # In the same Intersection
        #             for i in range(len(self.intersections)):
        #                 intersection = self.intersections[i]
        #                 self.dgraph[intersection] = {}
        #                 for j in range(len(intersection.direction_nodes)):
        #                     inodes = intersection.direction_nodes[j]
        #                     for jj in range(len(inodes)):
        #                         inode = inodes[jj]
        #                         if link.in_node is inode:
        #                             self.dgraph[intersection][link.out_node] = link.cost

        sink_node_list = []                    
        for i in range(len(self.sinks)):
            sink = self.sinks[i]
            self.dgraph[sink] = {}
            for j in range(len(sink.node_groups)):
                nodes = sink.node_groups[j]
                for jj in range(len(nodes)):
                    node = nodes[jj]       
                    for k in range(len(self.roads)):
                        graph_road = self.roads[k]
                        linkg = graph_road.link_groups
                        for l in range(2):
                            for link in linkg[l]:
                                if link.in_node is node:                                    
                                    self.dgraph[sink][link.out_node] = link.cost
                                    sink_node_list.append(node)
        #print(sink_node_list)


        for i in range(len(self.roads)):
            graph_road = self.roads[i]
            linkg = graph_road.link_groups
            for j in range(2):
                for link in linkg[j]:
                    self.dgraph[link.in_node] = {}
                    for k in range(len(sink_node_list)):
                        if link.in_node is not sink_node_list[k]:
                            self.dgraph[link.in_node][link.out_node] = link.cost
                                    

        return self.dgraph


        
    def createGridNetwork(self, N, num_lane):
        intersections = [[self.addIntersection('I'+str(i)+'_'+str(j), num_lane) for j in range(N)] for i in range(N)]
        
        sinks = [[self.addSink('S'+str(i)+'_'+str(j), num_lane) for j in range(N)] for i in range(4)]
        
        # Connect intersections
        for i_row in range(N):
            for i_col in range(1, N):
                component_1 = intersections[i_row][i_col-1]
                component_2 = intersections[i_row][i_col]
                self.connect(component_1, 2, component_2, 0)
        for i_col in range(N):
            for i_row in range(1, N):
                component_1 = intersections[i_row-1][i_col]
                component_2 = intersections[i_row][i_col]
                self.connect(component_1, 3, component_2, 1)
                
        # Connect sinks
        for i_idx in range(N):
            target_sink = sinks[0][i_idx]
            target_intersection = intersections[0][i_idx]
            self.connect(target_sink, 0, target_intersection, 1)
            
            target_sink = sinks[1][i_idx]
            target_intersection = intersections[i_idx][N-1]
            self.connect(target_sink, 0, target_intersection, 2)
            
            target_sink = sinks[2][i_idx]
            target_intersection = intersections[N-1][N-1-i_idx]
            self.connect(target_sink, 0, target_intersection, 3)
            
            target_sink = sinks[3][i_idx]
            target_intersection = intersections[N-1-i_idx][0]
            self.connect(target_sink, 0, target_intersection, 0)
    
        self.compile()
        
        return self
        
        
    def compile(self):
        self.is_compiled = True
        
    
    # ================ Run the network ===================