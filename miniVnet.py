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
                
    def checkSetting(self):
        component_list = [component for component in self.components if component != None]
        
        assert len(component_list) == 2, "Error: Road " + str(self.name) + " has too few connections."


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
        
        
    def compile(self):
        self.is_compiled = True
        
    
    # ================ Run the network ====================




