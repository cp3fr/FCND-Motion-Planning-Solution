from enum import Enum
from queue import PriorityQueue
import numpy as np


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


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
    pruned_path = [tuple(p) for p in pruned_path]

    return pruned_path


def grid_goal_verification(p, s, g, r=[10, 20, 40, 80]):

    #Check whether goal location is inside the grid
    if (p[0]<0 or p[0]>g.shape[0]-1 or
        p[1]<0 or p[1]>g.shape[1]-1 or
        g[p[0]][p[1]]):
        print('\nError: Specified goal is invalid. Looking for nearby location..\n')
        
        is_valid = False
        i = 0
        while is_valid==False or i<len(r):
            new_p = find_valid_grid_location(p, g, r = r[i])
            if len(new_p)>0:
                p=new_p
                is_valid = True
            i+=1
        if is_valid:
            print('..found a valid goal: {}'.format(p))
        else:
            p = s
            print('..found no valid goal. Resetting goal to current location: {}'.format(p))

    return p


def find_valid_grid_location(p, g, r = 10):

    corners = [p[0]-r, p[0]+r,
               p[1]-r, p[1]+r]

    if corners[0]<0:
        corners[0]=0

    if corners[0]>g.shape[0]-1:
        corners[0]=g.shape[0]-1

    if corners[1]<0:
        corners[1]=0

    if corners[1]>g.shape[0]-1:
        corners[1]=g.shape[0]-1

    if corners[2]<0:
        corners[2]=0

    if corners[2]>g.shape[1]-1:
        corners[2]=g.shape[1]-1

    if corners[3]<0:
        corners[3]=0

    if corners[3]>g.shape[1]-1:
        corners[3]=g.shape[1]-1

    locs = {}

    for i in range(corners[0], corners[1] + 1):
        for j in range(corners[2], corners[3] + 1):
            if g[i][j]==False:
                locs[(i,j)]=np.linalg.norm(np.array([i, j]) - np.array(p))

    if len(locs)>0:
        locs = {k: v for k, v in sorted(locs.items(), key=lambda item: item[1])}
        return list(locs.keys())[0]
    else:
        return []


