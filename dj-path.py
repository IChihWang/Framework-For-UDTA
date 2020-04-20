graph = {'a':{'b':10,'c':3},'b':{'c':1,'d':2},'c':{'b':4,'d':8,'e':2},'d':{'e':7},'e':{'d':9}}
print(graph)
from miniVnet import MINIVNET
inf = float('inf')

my_net = graph
#my_net = MINIVNET()
def dijkstra(my_net,start,goal):
    shortest_distance = {}
    predecessor = {}
    unseenNodes = graph
    infinity = inf
    path = []
    # set every node to be inf. Except the start node= 0
    for node in unseenNodes:
        shortest_distance[node] = infinity
    shortest_distance[start] = 0

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
            if childNode in unseenNodes:
                if weight + shortest_distance[minNode] < shortest_distance[childNode]:
                    shortest_distance[childNode] = weight + shortest_distance[minNode]
                    predecessor[childNode] = minNode
        unseenNodes.pop(minNode)

    currentNode = goal
    while currentNode != start:
        try:
            path.insert(0,currentNode)
            currentNode = predecessor[currentNode]
        except KeyError:
            print('Path not reachable')
            break
    path.insert(0,start)
    if shortest_distance[goal] != infinity:
        print('Shortest distance is ' + str(shortest_distance[goal]))
        print('And the path is ' + str(path))


dijkstra(graph, 'a', 'b')