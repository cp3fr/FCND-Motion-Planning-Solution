## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

I will focus on describing the starter code in plan_path() and planning_utils.py, which are called after the quadrotor has been armed.
In plan_path(), two hardcoded variables specify the flight altitude and the minimum safety distance to obstacles:

    TARGET_ALTITUDE = 5
    SAFETY_DISTANCE = 5

Next, obstacle information is read from a .csv file:

    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

These data are used for creating a 2D grid representation of configuration space at the target altitude and adding the specified safety distance to obstacle borders. Note that a fixed 1x1 m cell size is used:

    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

The grid variable represents configuration space, where free space is marked as False and occupied space as True. north_offset and east_offset represent the grid offset in local ECEF coordinates.

Next, the start location is set to the grid center location, and the goal to a location 10 meters north and east of that location

    grid_start = (-north_offset, -east_offset)
    grid_goal = (-north_offset + 10, -east_offset + 10)

Subsequently, the path from start to goal within the 2D grid is planned using A* :

    path, _ = a_star(grid, heuristic, grid_start, grid_goal)

using as a heuristic the euclidian distance:

    def heuristic(position, goal_position):
        return np.linalg.norm(np.array(position) - np.array(goal_position))

and allowing as actions only 1-m translations to adjacent cells in either west, east, north, or south direction:

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

Because no diagonal actions are allowed, the planned path follows a zig-zag trajectory with many waypoints to the goal.

No path pruning is applied.

After that, the 2D waypoints in grid space are converted to 3D waypoints in ECEF local coordinates, by adding north_offset and east_offset to x an y repectively, and using the target altitude for z. Also, heading for all waypoints specified as a zero angle with respect to the north direction:

    waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]

Finally, the waypoints are send to the simulator and the drone transitions to takeoff.



### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

I read colliders.csv as string and convert the values in the first row to latitude and longitude as float:

    (lat0, lon0) = [ np.float64(s.split(' ')[1]) for s in 
    np.loadtxt('colliders.csv', delimiter='\n', dtype='str')[0].split(', ') ]
        
Next, I set the global home position to these latitude and longitude coordinates and an altitude of zero (at the ground):

    self.set_home_position(lon0, lat0, 0.)




#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

I can either use *self.local_position* or the  *global_to_local()* function from frame_utils.py to convert current global position to local position:

    local_position = global_to_local((self._longitude, self._latitude, self._altitude), self.global_home)

    

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

I set grid_start to the current local position, which needs to be given as integer values to work:

    grid_start = (int(self.local_position[0] - north_offset), int(self.local_position[1] - east_offset))



#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

Taking some longitude and latitude location as input, i convert these geodetic coordinates, together with the target altitude to local coordinates, and subsequently to grid coordinates: 

    (lat, lon) =(37.793448, -122.398147)
    local_goal = global_to_local((lon, lat, -TARGET_ALTITUDE), self.global_home)
    grid_goal = (int(local_goal[0] - north_offset), int(local_goal[1] - east_offset))

In order to make sure that the specified goal location is valid, i created a method that checks the grid location, and if it collides with occupied space, searches for the nearest unoccupied location and sets this as the corrected goal location:

    grid_goal = grid_goal_verification(grid_goal, grid_start, grid)  

And here the function implemented in planning_utils.py:

    def grid_goal_verification(p, s, g, r=[10, 20, 40, 80]):
        
      if (p[0]<0 or p[0]>g.shape[0]-1 or
          p[1]<0 or p[1]>g.shape[1]-1 or
          g[p[0]][p[1]]):
          
          print('Invalid goal location. Looking for nearby location..')
          
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



#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

I simply added four diagonal actions to the Action class:

    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

and added conditional checks for removing those actions from the valid_actions list in the valid_actions function:

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
      valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
      valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
      valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
      valid_actions.remove(Action.SOUTH_EAST)



#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

I wrote a function for remove waypoints using collinearity checks:

     path = path_pruning(path, epsilon=0.1)

And here the function implemented in planning_utils.py. I iteratively check three subsequent points for collinearity and given a certain threshold (epsilon) remove the middle point:

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



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


