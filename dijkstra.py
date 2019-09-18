# single vehicle has 3 rodes to choose

# Sink is the node
# Road/num of lanes is the edge which has the cost
# Graph
# Compute the cost as a node not the upper level such as Intersection so on so forth
# look up the cost of edges by the lane dynamically

inf = float('inf')

import miniVnet

## Find the neighbors for each node
def neighbours(self):
    neighbours = {node: set() for node in self.node_groups}
    for edge in self.out_links:
        neighbours[edge.in_node].add((edge.out_node, edge.cost))
    return neighbours


def dijkstra(n, source, dest):
    
    assert source in self.node_groups, 'Such source node doesn\'t exist'

    distances = {node: inf for node in self.node_groups}
    previous_node = {node: None for node in self.node_groups}
    distances[source] = 0
    nodes = self.node_groups.copy()
    
    while nodes:
        current_node = min(nodes, key=lambda node: distances[node])

        if distances[current_node] == inf:
            break
        
        for neighbour, cost in self.neighbours[current_node]:
            alternative_route = distances[current_node] + cost

            if alternative_route < distances[neighbour]:
                distances[neighbour] = alternative_route
                previous_node[neighbour] = current_node
        
        nodes.remove(current_node)

    #path, current_node = deque(),dest
    #while previous_nodes[current_node] is not None:
    #    path.appendleft(current_node)
    #    current_node = previous_nodes[current_node]
    #if path:
    #    path.appendleft(current_node)
    # return path