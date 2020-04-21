from basicGraph import NODE, LINK, CAR
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
        self.value = 0
        

    def connect(self, index, road):
        assert self.components == None, "The sink is already connected, sink: " + str(self.name)
        
        assert index == 0, "Index out of range, sink: " + str(self.name)
        
        self.components = road
                    
        road.connect(self, self.node_groups[0], self.node_groups[1])
    def setValue(self, value):
        self.value = value

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
        self.nodes = []
        self.dgraph = {}
        self.intersection_nodes = []
        self.sink_nodes = []
        self.car_number = 0

    
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
        pass
    
    # =========== Use the DataStructure to route ====================== #                    
    def getNode(self, out_edge):
        return out_edge.out_node
    # Get the Intersection number from the number of the node number for finding the intersection to update cost value
    def sink_to_node(self, sink):    
        sink_node_list = []
        for nodes in sink.node_groups: 
            for sink_node in nodes:
                sink_node_list.append(sink_node)
        return sink_node_list             
    def node_to_intersetion(self, node):
        for intersection in self.intersections:
            for j in range(len(intersection.direction_nodes)):
                inodes = intersection.direction_nodes[j]
                for inode in inodes:
                    for insec_node in inode:
                        if node is insec_node:
                            return intersection

    def dijkstra(self,car_list,s1,s2, global_time):
        car = CAR()
        #self.car_number += 1
        #car_id = self.car_number + 1
        self.start = self.sinks[s1]
        self.goal = self.sinks[s2]
        print('--------- Start Routing ---------')
        print('Routing the path goes from {} to {}'.format(self.start,self.goal))
        self.potential_list = []
        self.from_node = {}
        self.from_edge = {}
        self.car_timestamp = {}
        # Reset
        for node in self.nodes:
            node.setValue(inf)
        # set every node to be inf. Except the start node= 0
        start_nodes = self.sink_to_node(self.start)
        for node in start_nodes:
            node.setValue(0)
        
        self.node_list = []
        for snode in start_nodes:
            self.node_list.append(snode)
        # algorithm. Check the dictionary is empty or note
        while self.node_list:
            # Greedy. Lowset node for this
            node = self.node_list.pop()
            time = node.value
            # Only intersection nodes have the turning information
            if node in self.intersection_nodes:
                if node.id % 4 == 1:
                    car_list['N'].append(car)
                elif node.id % 4 == 2:
                    car_list['E'].append(car)
                elif node.id % 4 == 3:
                    car_list['S'].append(car)
                else:
                    car_list['W'].append(car)
            
                for out_edge in node.out_links:
                    out_node = self.getNode(out_edge)
                    if out_node.id % 3 == 0:
                        car.turning = 'L'
                    elif out_node.id % 3 == 1:
                        car.turning = 'S'
                    else: 
                        car.turning = 'R'
                    # get cost from RoadRunner
                    self.next_time = node.value + car_cost['L']
                    if self.next_time < out_node.value:
                        out_node.setValue(self.next_time)
                        self.node_list.append(out_node)
                        self.from_node[out_node] = node
                        self.from_edge[out_node] = out_edge
                
                self.car_timestamp[node] = node.value

                    #for _ in range(time - 9):
                        # append a constant value: 1
                    #    out_edge.cost.append(0)
                    #self.next_time = node.value + out_edge.cost[time]
                    
            for out_edge in node.out_links:
                out_node = self.getNode(out_edge)
                # get cost from RoadRunner
                car_cost = get_cost(car_list)
                self.next_time = node.value + car_cost[]
                if self.next_time < out_node.value:
                    out_node.setValue(self.next_time)
                    self.node_list.append(out_node)
                    self.from_node[out_node] = node
                    self.from_edge[out_node] = out_edge
            self.car_timestamp[node] = node.value
        
        # Record the path
        self.goal_nodes = self.sink_to_node(self.goal)
        self.path = []
        self.path_node_list = []
        self.path_edge_list = []
        for gnode in self.goal_nodes:
            if gnode in self.from_node:
                self.path_node_list.append(gnode)
                while self.path_node_list:
                    try:
                        self.path_node = self.path_node_list.pop()
                        self.path.insert(0, self.path_node)
                        self.path_node_list.append(self.from_node[self.path_node])
                        self.path_edge_list.insert(0, self.from_edge[self.path_node])
                    except KeyError:
                        break
        for node in self.path:
            self.potential_list.append(self.car_timestamp[node])
        minTime = self.potential_list[-1]
        t = len(self.potential_list)
        for x in range(len(self.potential_list) - 1):
            if (self.potential_list[x+1] == 0) & (self.potential_list[x] <= minTime) :
                minTime = self.potential_list[x]
                t = x
        self.optimaltime = []
        self.optimallist = []
        while self.potential_list[t-1] != 0:
            self.optimallist.insert(0, self.path[t-1])
            self.optimaltime.insert(0, self.potential_list[t-1])
            t = t - 1
        self.optimallist.insert(0,self.path[t-1])
        self.optimaltime.insert(0, self.potential_list[t-1])
        #print(self.path, self.potential_list)
        self.print_list = []
        self.print_list.append(self.start)
        for node in self.optimallist[1:-1]:
            self.print_list.append(self.node_to_intersetion(node))
        self.print_list.append(self.goal)

        time_offset = global_time
        for i in range(1, len(self.optimaltime)):
            remaining_time = self.optimaltime[i]
            for j in range(time_offset):
                if len(self.path_edge_list[i - 1].at) <= j:
                    self.path_edge_list[i - 1].at.append([])

            for j in range(time_offset, time_offset + remaining_time):
                if len(self.path_edge_list[i - 1].at) <= j:
                    self.path_edge_list[i - 1].at.append([])
                self.path_edge_list[i - 1].at[j].append((car_id, time_offset + remaining_time - j))
            time_offset = time_offset + remaining_time

        print('The car is ' + str(car_id))
        print('The path is ' + str(self.print_list))
        print('Coresponding time is' + str(self.optimaltime))
        print('The detailed link is' + str(self.path_edge_list))

        for link in self.path_edge_list:
            print(link.at)
        print('# ============================= #')




        return self.optimallist, self.print_list, self.optimaltime
        #     self.path.insert(0,start)
        #     if goal_node.value != infinity:
        #     self.potential_list = self.potential_list[::-1]
        #     print(self.potential_list)
        # return self.path, self.potential_list

    #======================= UPDATE The Intersection Part ===============================#
    
    def updateLinkCost(self,path,time):
        for i in range(0,len(path)-1,2):
            for link in path[i].out_links:
                if link.out_node is path[i+1]:
                    link.cost[time[i]] += 1
    
    def resetLinkCost(self,path,time):
        for i in range(0,len(path)-1,2):
            for link in path[i].out_links:
                if link.out_node is path[i+1]:
                    link.cost[time[i]] -= 1

    def updateIntersection(self, path,time):
        intersection_list = self.intersection_nodes
        self.timelist = {}
        for i in range(len(path)):
            node = path[i]
            #print(node.value)
            if node in intersection_list:
                self.timelist[time[i]]  = self.node_to_intersetion(node)
        return self.timelist
    def mergeDict(self, dict1, dict2):
        dict3 = {**dict1, **dict2}
        for key, value in dict3.items():
            if key in dict1 and key in dict2:
                dict3[key] = [value , dict1[key]]
        return dict3
# Merge dictionaries and add values of common keys in a list
    def multipleIntCount(self, paths,times):
        time_dic = {}
        for i in range(len(paths)):
            #time_dic.update(self.updateIntersection(paths[i],times[i]))
            time_dic = self.mergeDict(time_dic,self.updateIntersection(paths[i],times[i]))
        time_dic = dict(sorted(time_dic.items()))
        print(time_dic)
            
    
    
    # ============= Add Links to the Network ====================== #    
    def appendLinktoIntersection(self):
        for graph_intersection in self.intersections:
            # Add link inside Intersections
            for i in range(4):
                for j in range(graph_intersection.num_lane):
                    for k in range(3):
                        graph_intersection.new_link[i][j][k].in_node.out_links.append(graph_intersection.new_link[i][j][k])
        
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
                                            link.in_node.out_links.append(link)
    def appendLinktoSink(self):
        for sink_node in self.sink_nodes:
            for graph_road in self.roads:
                linkg = graph_road.link_groups
                for l in range(2):
                    for link in linkg[l]:     
                        if link.in_node is sink_node:
                            link.out_node.in_links.append(link)
                            sink_node.out_links.append(link)
                        elif link.out_node is sink_node:
                            link.in_node.out_links.append(link)
                            sink_node.in_links.append(link)

    # ======================== Compile ========================== #
    def compile(self):
        self.is_compiled = True
        # Network list
        for sink in self.sinks:
            for nodes in sink.node_groups: 
                for sink_node in nodes:
                    self.sink_nodes.append(sink_node) 
        for intersection in self.intersections:
            for j in range(len(intersection.direction_nodes)):
                inodes = intersection.direction_nodes[j]
                for inode in inodes:
                    for insec_node in inode:
                        self.intersection_nodes.append(insec_node)
        for graph_road in self.roads:
            linkg = graph_road.link_groups
            for l in range(2):
                for link in linkg[l]:    
                    self.nodes.append(link.in_node)
                    self.nodes.append(link.out_node)
        self.nodes = list(set(self.nodes)) 
        self.appendLinktoSink()
        self.appendLinktoIntersection()
        
    
    # ================ Run the network ===================