from basicGraph import NODE, LINK
import itertools
inf = float('inf')


class INTERSECTION:
    def __init__(self, name, num_lane):
        self.name = name
        self.num_lane = num_lane
        self.components = [None for i in range(4)]
        self.direction_nodes = [[[NODE() for k in range(num_lane)] for j in range(2)] for i in range(4)]
        #self.new_link = []
        self.new_link = [[[LINK() for k in range(3)] for j in range(num_lane)] for i in range(4)]
        #Allocate links to nodes
        for i in range(4):
            for j in range(num_lane):
                for k in range(3):  # left, straight, right
                    source_node = self.direction_nodes[i][0][j]   
                    #new_link = LINK()
                    #source_node.out_links.append(new_link)
                    source_node.out_links.append(self.new_link[i][j][k])
                    #new_link.in_node = source_node
                    self.new_link[i][j][k].in_node = source_node                                        
                    #self.new_link.append(source_node)

                    sink_node = self.direction_nodes[(i+1+k)%4][1][j]
                    #sink_node.in_links.append(new_link)
                    sink_node.in_links.append(self.new_link[i][j][k])
                    #new_link.out_node = sink_node
                    self.new_link[i][j][k].out_node = sink_node
                    #self.new_link.append(sink_node)

                    
        
        
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
    
    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.name)

        
            


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
    newid = itertools.count(0) 
    def __init__(self, name, num_lane):
        self.name = name
        self.num_lane = num_lane
        self.node_groups = [[NODE() for i in range(num_lane)] for i in range(2)]
        self.components = None
        self.id = next(SINK.newid)

    def connect(self, index, road):
        assert self.components == None, "The sink is already connected, sink: " + str(self.name)
        
        assert index == 0, "Index out of range, sink: " + str(self.name)
        
        self.components = road
                    
        road.connect(self, self.node_groups[0], self.node_groups[1])

    def __repr__(self):
        return '{} {}'.format(self.__class__.__name__, self.name)

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
        
    
    # ==================== Routing For Cars ======================= #
    # Debug Purpose
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
        
        print(self.intersections[0].new_link)
        # for road in self.roads:
        #     linkg = road.link_groups
        #     linkg[0]
      #  for i in range(len(self.roads)):
       #     graph_road = self.roads[i]
        #    linkg = graph_road.link_groups
         #   print(linkg)
            #for j in range(2):
            #    for link in linkg[j]:
            #        print(link.in_node)
    # Next task
    def node_to_intersetion(self,node):
        for intersection in self.intersections:
            for j in range(len(intersection.direction_nodes)):
                inodes = intersection.direction_nodes[j]
                for inode in inodes:
                    for insec_node in inode:
                        if node is insec_node:
                            return intersection
                        
    # Next task            
    def sink_node(self,graph_sink):
        print(graph_sink.node_groups)
        sink_node = {}
        for j in range(len(graph_sink.node_groups)):
            sink_nodes = graph_sink.node_groups[j]
            for jj in range(len(sink_nodes)):
                sink_node[graph_sink] = sink_nodes[jj]
        print(sink_node)

    # Transfer the network graph into 
    def dijkstraGraph(self):
        for graph_intersection in self.intersections:
            # Add link inside Intersections
            for i in range(4):
                for j in range(graph_intersection.num_lane):
                    self.dgraph[graph_intersection.new_link[i][j][0].in_node] = {}
                    for k in range(3):
                        link = graph_intersection.new_link[i][j][k]
                        self.dgraph[link.in_node][link.out_node] = link.cost
            # Add link between Intersections
            for j in range(len(graph_intersection.direction_nodes)):
                inodes = graph_intersection.direction_nodes[j]
                for jj in range(len(inodes)):
                    inode = inodes[jj]
                    for insec_node in inode:
                        for graph_road in self.roads:
                            linkg = graph_road.link_groups
                            for l in range(2):
                                for link in linkg[l]:         
                                    if link.in_node is insec_node:
                                        self.dgraph[link.in_node] = {}
                                        self.dgraph[link.in_node][link.out_node] = link.cost
        #sink_node_list = []  
        #print(sink_node_list)     
        # Add link in/out Sink
  
        for graph_sink in self.sinks:
            self.dgraph[graph_sink] = {}
            for j in range(len(graph_sink.node_groups)):
                nodes = graph_sink.node_groups[j]
                for jj in range(len(nodes)):
                    sink_node = nodes[jj]
                    for graph_road in self.roads:
                        linkg = graph_road.link_groups
                        for l in range(2):
                            for link in linkg[l]:     
                                if link.in_node is sink_node:
                                    self.dgraph[graph_sink][link.out_node] = link.cost
                                elif link.out_node is sink_node:
                                    self.dgraph[link.in_node] = {}                                    
                                    self.dgraph[link.in_node][graph_sink] = link.cost
                                # else:
                                #     self.dgraph[link.in_node] = {}
                                #     self.dgraph[link.in_node][link.out_node] = link.cost

        return self.dgraph
    
    # Modified Dijkstra's Algorithm
    def dijkstra(self,s1,s2):
        graph = self.dijkstraGraph()        
        start = self.sinks[s1]
        goal = self.sinks[s2]
        print('Return the path goes from {} to {}'.format(start,goal))
        #Initialization
        shortest_distance = {}
        predecessor = {}
        unseenNodes = graph.copy()
        infinity = inf
        self.path = []
        timelist = {}
        printtimelist = {}
        printtimelist[start] = 0
        # set every node to be inf. Except the start node= 0
        for node in unseenNodes:
            shortest_distance[node] = infinity
        shortest_distance[start] = 0
        self.time = 0
        # algorithm. Check the dictionary is empty or note
        while unseenNodes:
            # Greedy. Loeset node for this
            minNode = None
            for node in unseenNodes:
                if minNode is None:
                    minNode = node
                elif shortest_distance[node] < shortest_distance[minNode]:
                    minNode = node

            for childNode, weight in unseenNodes[minNode].items():                
                for _ in range(self.time - 9):
                    weight.append(0)
                if childNode in unseenNodes:
                    if weight[self.time] + shortest_distance[minNode] < shortest_distance[childNode]:
                        shortest_distance[childNode] = weight[self.time] + shortest_distance[minNode]
                        predecessor[childNode] = minNode
                        timelist[childNode] = self.time
                        self.time += weight[self.time]
            unseenNodes.pop(minNode)
        
        # Record the path
        currentNode = goal
        while currentNode != start:
            try:
                self.path.insert(0,currentNode)
                currentNode = predecessor[currentNode]
            except KeyError:
                print('Path not reachable')
                break
        self.path.insert(0,start)
        if shortest_distance[goal] != infinity:
        # print('Shortest distance is ' + str(shortest_distance[goal]))
            print('And the path is ' + str(self.path))
        # for i in self.path:
        #     if i != start:
        #         printtimelist[i] = timelist[i]     
        # print(printtimelist)
        return self.path

    def newdijkstra(self,s1,s2):
        graph = self.dijkstraGraph()
        start = self.sinks[s1]
        goal = self.sinks[s2]
        print('Return the path goes from {} to {}'.format(start,goal))
        shortest_distance = {}
        unseenNodes = graph
        infinity = inf
        self.path = []
        self.print_time_list = []
        time = 0
        from_node = {}
        car_timestamp = {}

        # set every node to be inf. Except the start node= 0
        for node in unseenNodes:
            shortest_distance[node] = infinity
        shortest_distance[start] = 0
        node_list = []
        node_list.append(start)
        
        # algorithm. Check the dictionary is empty or note
        while node_list:
            # Greedy. Loeset node for this
            minNode = node_list.pop()
            for childNode, weight in unseenNodes[minNode].items():
                time = shortest_distance[minNode]
                for _ in range(time - 9):
                    weight.append(0)
                next_time = shortest_distance[minNode] + weight[time]
                if next_time < shortest_distance[childNode]:
                    shortest_distance[childNode] = next_time
                    node_list.append(childNode)
                    from_node[childNode] = minNode
            car_timestamp[minNode] = time
        # Record the path
        currentNode = goal
        while currentNode != start:
            try:
                self.path.insert(0,currentNode)
                currentNode = from_node[currentNode]
                print_time_list.append(car_timestamp[currentNode])
            except KeyError:
                print('Path not reachable')
                break
        self.path.insert(0,start)
        if shortest_distance[goal] != infinity:
            print('And the path is ' + str(self.path))
        self.print_time_list = print_time_list[::-1]
        print(slef.print_time_list)
        return self.path, self.print_time_list

    #======================= UPDATE The Intersection Part ===============================#
    # Need to be done by Nov.7
    def updateIntersection(self, graph, path):
        self.time = {}
        t = 0
        for i in range(0,len(path),2):

            try:
                t += graph[path[i]][path[i+1]]
                self.time[t] = self.node_to_intersetion(path[i+1])
            except:
                t += graph[path[i]]
                self.time[t] = self.node_to_intersetion(path[i])
        print(self.time) 
        #self.time + = 
        
        
    def compile(self):
        self.is_compiled = True
        
    
    # ================ Run the network ===================