from enum import Enum
from queue import PriorityQueue
import numpy as np
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString
import networkx as nx
from scipy.spatial import Voronoi
from bresenham import bresenham


class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]
    
    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)

    def disjoint(self, other):
        return self._polygon.disjoint(other)



def extract_polygons(data, dist):

    polygons = []
    centers = []
    
    for i in range(data.shape[0]):

        # print('obstacle {}/{}'.format(i,data.shape[0]))
        
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        obstacle = [north - d_north - dist, 
                    north + d_north + dist, 
                    east - d_east - dist, 
                    east + d_east + dist]
        
        corners = [(obstacle[0], obstacle[2]), 
                   (obstacle[0], obstacle[3]), 
                   (obstacle[1], obstacle[3]), 
                   (obstacle[1], obstacle[2])]
        
        height = alt + d_alt

        p = Poly(corners, height)
        
        polygons.append(p)
        centers.append((north,east))

    return (polygons,centers)




class Sampler:

    def __init__(self, data, alt, dist):
  
        # print('....getting polygons and centers')
        self._polygons, centers = extract_polygons(data, dist)

        # print('....setting limits')
        self._xmin = np.min(data[:, 0] - data[:, 3] - dist)
        self._xmax = np.max(data[:, 0] + data[:, 3] + dist)

        self._ymin = np.min(data[:, 1] - data[:, 4] - dist)
        self._ymax = np.max(data[:, 1] + data[:, 4] + dist)

        if isinstance(alt,int):
            self._zmin = alt
            self._zmax = alt
        else:
            self._zmin = alt[0]
            self._zmax = alt[1]


        # print('....computing max_poly_xy')
        # Record maximum polygon dimension in the xy plane
        # multiply by 2 since given sizes are half widths
        # This is still rather clunky but will allow us to 
        # cut down the number of polygons we compare with by a lot.
        self._max_poly_xy = 2 * np.max((data[:, 3], data[:, 4])) 
  
        # print('..getting centers')
        # centers = np.array([p.center for p in self._polygons])
 
        # print('....making center tree')
        self._tree = KDTree(centers, metric='euclidean')

        
    def sample(self, num_samples):
        """Implemented with a k-d tree for efficiency."""

        pts = []

        while len(pts)<num_samples:

            xvals = np.random.uniform(self._xmin, self._xmax, num_samples*2)
            yvals = np.random.uniform(self._ymin, self._ymax, num_samples*2)
            zvals = np.random.uniform(self._zmin, self._zmax, num_samples*2)
            samples = list(zip(xvals, yvals, zvals))

            for s in samples:

                in_collision = False
                
                #query for neighbors within a given radius
                idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), 
                                                    r=self._max_poly_xy)[0])
                if len(idxs) > 0:
                    for ind in idxs: 
                        p = self._polygons[int(ind)]
                        if p.contains(s) and p.height >= s[2]:
                            in_collision = True
                if not in_collision:
                    pts.append(s)

                if len(pts)>=num_samples:
                    return pts
                
        return pts

    @property
    def polygons(self):
        return self._polygons



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



#method for creating the graph
def create_graph(nodes, k, sampler):

    g = nx.Graph()
    
    #tree of nodes
    node_tree = KDTree(nodes)

    #loop over nodes
    for n1 in nodes:

        #find k nearest nodes
        node_idx = node_tree.query([n1], k, return_distance=False)[0]
        
        #loop over nearest nodes
        for i in node_idx:

            #current nearest node
            n2 = nodes[i]

            #check if nodes are identical
            if not n1==n2:

                #check if edge already exists
                if not g.has_edge(n1, n2):

                    #assume nodes are connectable by default
                    is_connectable = True

                    #discrete points connecting the points
                    pts = list(bresenham(int(n1[0]), int(n1[1]), int(n2[0]), int(n2[1])))

                    #loop over a selection of points (every 10 meters) and check for collision with nearest polygon
                    for ind in range(0, len(pts), 20 ):

                        #current point on the line
                        p = pts[ind]

                        #find closest polygon to the current point
                        j = sampler._tree.query([p], 1, return_distance=False)[0][0]

                        #check if closest polygon and point of the line overlap
                        if sampler._polygons[j].disjoint(Point(p))==False:

                            #if they do, set connectable to false
                            is_connectable = False
                            break

                    #if connectable, add edge
                    if is_connectable:
                        g.add_edge(n1, n2, weight=1)

    return g



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))



#method for running A* on a graph
def a_star_graph(graph, h, start, goal):
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
                new_cost = current_cost + cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
                    branch[next_node] = (new_cost, current_node)
             
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
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
    pruned_path = [tuple(p) for p in pruned_path]

    return pruned_path



def waypoints_from_path(path, local_position, heading=True):
    '''
    Generates a list of waypoints [ [int(x), int(y), int(z), angle], ... ] from
    path:  3d grid coordinates in a list [ (x, y, z), ...]
    local_position:   current local position 
    heading:   whether or not to orient the quadrotor towards the next waypoint
    '''

    waypoints = []

    #loop over waypoints
    for i in range(len(path)):
        
        #previous location
        if i==0:  
            p0 = np.array([ local_position[0], local_position[1] ]) 
        else:
            p0 = np.array([ path[i-1][0], path[i-1][1] ]) 

        #current location
        p1 = np.array([ path[i][0], path[i][1]])  

        #unit vector from translation from previous to current location
        v01 =  p1 - p0               
        v01 = v01 / np.linalg.norm(v01)

        #unit vector pointing into north direction
        vref = np.array([1.,0.])     
        vref = vref / np.linalg.norm(vref)

        #angle between translation vector and north (note: it's the smallest angle, between 0-pi radians or 0-180 deg)
        angle = np.arccos(np.dot(vref, v01))

        #check rotation direction: clockwise/positive (default) for translations to the east and
        #counterclockwise/negative for translations to the wests
        if v01[1]<0:
            angle = -angle

        #append waypoint locations (integers) and rotation angle (in radians, relative to north)
        waypoints.append( tuple([int(path[i][0]), int(path[i][1]), int(path[i][2]), angle]) )


    return waypoints



def update_local_map(voxmap, local_position, sampler, voxel_size=1):
    '''
    Function that updates an existing voxmap centered on a local NED position
    using obstacle information in sampler
    '''

    #diameter along the three axes
    (xd, yd) = voxmap.shape

    #clear the voxmap
    voxmap = np.zeros((xd, yd), dtype=np.bool)

    # minimum and maximum north coordinates
    north_min =  local_position[0] - voxel_size * xd // 2
    north_max =  local_position[0] + voxel_size * xd // 2
    
    # minimum and maximum east coordinates
    east_min =  local_position[1] - voxel_size * yd // 2
    east_max =  local_position[1] + voxel_size * yd // 2
    
    #maximum distance between point and outer voxels
    d_voxmap = np.sqrt((xd//2)**2+(yd//2)**2) 
    
    #maximum distance between obstacle center and outer borders
    d_obstacle = sampler._max_poly_xy

    #maximum combined distances between voxmap center and obstacle centers
    d_max = d_voxmap + d_obstacle

    #all obstacles in vincinity
    idxs = list(sampler._tree.query_radius([local_position[:2]], r=d_max)[0])
    
    #loop over closeby obstacles
    for i in idxs:
        
        #current obstacle
        p = sampler._polygons[i]

        #check if obstacle height is bigger than current altitude
        if p.height >= -local_position[2]:
        
            #get the obstacle bounds (north_min, north_max, east_min, east_max)
            bounds = [
                np.min([vals[0] for vals in p.coords]),
                np.max([vals[0] for vals in p.coords]),
                np.min([vals[1] for vals in p.coords]),
                np.max([vals[1] for vals in p.coords])
            ]
            
            #discretize obstacle bounds according to voxel size
            obstacle = [
                int(bounds[0] - north_min) // voxel_size,
                int(bounds[1] - north_min) // voxel_size,
                int(bounds[2] - east_min) // voxel_size,
                int(bounds[3] - east_min) // voxel_size
            ]
            
            #correct for out-of-bound values
            if obstacle[0]<0:
                obstacle[0]=0
            elif obstacle[0]>voxmap.shape[0]-1:
                obstacle[0]=voxmap.shape[0]-1

            if obstacle[1]<0:
                obstacle[1]=0
            elif obstacle[1]>voxmap.shape[0]-1:
                obstacle[1]=voxmap.shape[0]-1

            if obstacle[2]<0:
                obstacle[2]=0
            elif obstacle[2]>voxmap.shape[1]-1:
                obstacle[2]=voxmap.shape[1]-1

            if obstacle[3]<0:
                obstacle[3]=0
            elif obstacle[3]>voxmap.shape[1]-1:
                obstacle[3]=voxmap.shape[1]-1

            #add collision information to the voxmap
            voxmap[obstacle[0]:obstacle[1]+1,
                   obstacle[2]:obstacle[3]+1] = True
       
    #return the voxmap
    return voxmap



def local_path_obstacle_detection(voxmap, v, voxel_size=1, deadband=0):

    #diameter along the three axes
    (xd, yd) = voxmap.shape

    #center voxel
    center = (int(xd//2), int(yd//2))

    #set goal position in voxmap space
    goal = ( 
        int(center[0] + v[0] // voxel_size), 
        int(center[1] + v[1] // voxel_size) 
        )

    #ray casting using bresenham
    pts = list(bresenham(center[0], center[1], goal[0], goal[1]))

    #only keep points within the grid
    pts = [ p for p in pts if p[0]>=0 and p[0]<xd and p[1]>=0 and p[1]<yd ]

    collision_point = []

    #loop over discretized points
    for i in range(deadband,len(pts)):

        p = pts[i]

        #check if it collides with an obstacle
        if voxmap[p[0], p[1]]:

            collision_point = p

            #if yes return obstacle in path=true
            return (True, pts, collision_point)

    #if no obstacle in path return=false
    return (False, pts, collision_point)



def print_local_map(voxmap, path):

    #make local path voxmap
    voxmap_local_path = np.zeros(voxmap.shape, dtype=np.bool)

    #add discretized local path points
    for p in path:
        voxmap_local_path[p[0],p[1]]=True

    print('========================')
    s=''
    for i in range(voxmap.shape[0]):
        s+='\n'
        for j in range(voxmap.shape[0]):

            if i==int(voxmap.shape[0]/2) and j==int(voxmap.shape[1]/2):
                s+='X'
            else:
                if voxmap_local_path[i,j]:
                    s+='P'
                else:
                    if voxmap[i,j]:
                        s+='o'
                    else:
                        s+='.'
    print(s)
    print('========================')















## in progress ...


def create_local_voxmap(sampler, point, xd=10, yd=10, zd=10, voxel_size=1):
    """
    Returns a grid representation of a 3D space
    around a 3d point
    
    'xd, yd, zd' arguments set the voxel radius along the three dimensions
    
    The `voxel_size` argument sets the resolution of the voxel map. 

    """
    
    # minimum and maximum north coordinates
    north_min =  point[0] - xd
    north_max =  point[0] + xd
    
    # minimum and maximum east coordinates
    east_min =  point[1] - yd
    east_max =  point[1] + yd
    
    # miniumum and maximum altitude
    alt_min =  point[2] - zd
    alt_max =  point[2] + zd
       
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min))) // voxel_size
    east_size = int(np.ceil((east_max - east_min))) // voxel_size
    alt_size = int(np.ceil((alt_max - alt_min))) // voxel_size

    # Create an empty grid
    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)
    
    #maximum distance between point and outer voxels
    d_voxmap = np.sqrt((xd**2+yd**2) + (zd/2)**2)
    
    #maximum distance between obstacle center and outer borders
    d_obstacle = np.max(np.array([ 
            LA.norm(np.array(p.coords[0]) - 
                    np.array(p.coords[2])) / 2 
            for p in polygons]))
    
    #maximum combined distances between voxmap center and obstacle centers
    d_max = d_voxmap + d_obstacle

    #all obstacles in vincinity
    idxs = list(sampler._tree.query_radius(point[:2], r=d_max))[0]
    
    #loop over closeby obstacles
    for i in idxs:
        
        #current obstacle
        p = polygons[i]
        
        #get the obstacle bounds (north_min, north_max, east_min, east_max)
        bounds = [
            np.min([vals[0] for vals in p.coords]),
            np.max([vals[0] for vals in p.coords]),
            np.min([vals[1] for vals in p.coords]),
            np.max([vals[1] for vals in p.coords]),
            0.,
            p.height
        ]
        
        #discretize obstacle bounds according to voxel size
        obstacle = [
            int(bounds[0] - north_min) // voxel_size,
            int(bounds[1] - north_min) // voxel_size,
            int(bounds[2] - east_min) // voxel_size,
            int(bounds[3] - east_min) // voxel_size,
            int(bounds[4] - alt_min) // voxel_size,
            int(bounds[5] - alt_min) // voxel_size
        ]
        
        #correct for out-of-bound values
        if obstacle[0]<0:
            obstacle[0]=0
        if obstacle[1]>voxmap.shape[0]-1:
            obstacle[1]=voxmap.shape[0]-1
        if obstacle[2]<0:
            obstacle[2]=0
        if obstacle[3]>voxmap.shape[1]-1:
            obstacle[3]=voxmap.shape[1]-1
        if obstacle[4]<0:
            obstacle[4]=0
        if obstacle[5]>voxmap.shape[2]-1:
            obstacle[5]=voxmap.shape[2]-1
            
        #add collision information to the voxmap
        voxmap[obstacle[0]:obstacle[1]+1,
               obstacle[2]:obstacle[3]+1,
               obstacle[4]:obstacle[5]+1] = True
        
        #collect collision information for the ground floor
        floor = int(0-alt_min)//voxel_size

        #if voxmap collides with ground floor: add collision information
        if floor>=0:
            voxmap[:,:,:floor]=True
    
    #return the voxmap
    return voxmap


def define_voxmap_goal(voxmap, point, voxel_size=1):
    
    #in voxmap coordinates
    center = np.array([int(val/2) for val in voxmap.shape])

    #voxel offset relative to center
    north_min = - voxmap.shape[0]/2
    north_max =  voxmap.shape[0]/2

    east_min = - voxmap.shape[1]/2
    east_max =  voxmap.shape[1]/2

    alt_min = - voxmap.shape[2]/2
    alt_max =  voxmap.shape[2]/2
    
    #by default use the center as next goal
    goal = center
    
    #loop over 1-unit multiplication factor to find the outer point
    #the basic idea is, we start with the original point and check if 
    #it is within the voxmap.
    #if not, we iteratively decrease the vector length until it hits
    #the outer boundary of the voxmap, which we take as the goal point
    #the multiplication factor can be fixed to 0.1 step
    #or made to vary as a fixed distance, e.g. 1 unit. then use -1/LA.norm(point)
    for m in np.arange(1., 0.1, -1/np.linalg.norm(point)):
    
        #current point
        current_point = [p*m for p in point]

        #in voxmap coordinates
        current_goal = [
            int(int(current_point[0]) // voxel_size - north_min),
            int(int(current_point[1]) // voxel_size - east_min),
            int(int(current_point[2]) // voxel_size - alt_min)
         ]

        #if goal inside the voxmap limits
        if  (current_goal[0]>=0 and  current_goal[0]<voxmap.shape[0] and
             current_goal[1]>=0 and  current_goal[1]<voxmap.shape[1] and
             current_goal[2]>=0 and  current_goal[2]<voxmap.shape[2]):

            goal = np.array(current_goal)
            break


    #check if goal in occupied space
    in_collision = voxmap[goal[0]][goal[1]][goal[2]]

    w = 0
    while in_collision:
        w+=1
        for i in range(goal[0]-w, goal[0]+w+1):
            for j in range(goal[1]-w, goal[1]+w+1):

                if i>=0 and i<voxmap.shape[0] and j>=0 and j<voxmap.shape[1]:

                    if voxmap[i][j][goal[2]]==False:
                        goal=(i,j,goal[2])
                        in_collision=False
                        return goal,center

    return goal, center



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

