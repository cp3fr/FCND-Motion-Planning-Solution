import numpy as np
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString
import networkx as nx
from enum import Enum
from queue import PriorityQueue

#method for checking if two nodes can connect
def can_connect(n1, n2, polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True

#method for creating the graph
def create_graph(nodes, k, polygons):
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue
                
            if can_connect(n1, n2, polygons):
                g.add_edge(n1, n2, weight=1)
    return g


#method to find the closest node in the graph
def closest_point(g, p):
    """
    Compute the closest point in the graph `g`
    to the current point `p`.
    """
    nodes = [n for n in g.nodes]
    tree = KDTree(nodes)
    idx = tree.query([p], k=1, return_distance=False)[0][0]
    return nodes[idx]


#method for euclidian heuristic
def heuristic(n1, n2):
    return np.linalg.norm(np.array(n2) - np.array(n1))


#method for running A* on a graph
def a_star_graph(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
                    branch[next_node] = (new_cost, current_node)
             
    print('..printing branch dict:')
    print(branch)
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        print('[ERROR]')
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
            
    return path[::-1], path_cost


def path_pruning(path, epsilon=1e-3):

    i=0
    pruned_path=[p for p in path]
    while i < len(pruned_path) - 2:

        det = np.linalg.det( np.concatenate((
            np.array([pruned_path[i  ][0], pruned_path[i  ][1], 1.]).reshape(1, -1),
            np.array([pruned_path[i+1][0], pruned_path[i+1][1], 1.]).reshape(1, -1),
            np.array([pruned_path[i+2][0], pruned_path[i+2][1], 1.]).reshape(1, -1)
            ), 0))

        if abs(det) < epsilon:
            pruned_path.remove(pruned_path[i+1])
        else:
            i +=1
    path = [tuple(p) for p in pruned_path]

    return path