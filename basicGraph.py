
class NODE:
    def __init__(self):
        #self.name = None
        self.is_activated = False
        self.in_links = []
        self.out_links = []
        self.value = None # For Dijkstra node value


class LINK:
    def __init__(self):
        #self.name = None
        self.in_node = None
        self.out_node = None
        self.cost = 0
    def insertCost(self,value):
        self.cost = value
