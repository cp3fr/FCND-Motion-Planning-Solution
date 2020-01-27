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
These scripts contain a basic planning implementation that includes:

(1) A grid representation of 2D configuration space with 1x1m cell size, based on obstacle information in 'colliders.csv' (motion_planning.py, line 133), for a 5-m altitude given by 'TARGET_ALTITUDE' (motion_planning.py, line 117) and considering a 5-m safety distance to obstacles given by 'SAFETY_DISTANCE' (motion_planning.py, line 118).

(2) A specification of start and goal locations within the 2D grid, where start is at grid center and goal is 10 m to the north and east of the start (motion_planning.py, lines 139 and 143).

(3) The use of A* for planning a trajectory in 2D configuration space from start to goal (planning_utils.py, line 45). 

(4) Possible actions are linear translations to adjacent cells in west, east, north, or south direction (planning_utils.py, lines 54-57) specified by three-number tuples: The first two values indicate the delta between current and next position in north and east direction (possible values: -1, 0, 1). The third value indicates the cost of performing the action (value of 1 in all cases).

(5) As a heuristic, manhattan distance is used (planning_utils.py, line 144).

(6) No path pruning (e.g. test for collinearity, bresenham algorithm) is applied. Instead the path from start to goal, identified by A* , consists of many waypoints in linearly adjacent cells (motion_planning.py, line 155). Because of this and because no diagonal actions are possible, there is a visible zig-zag trajectory along many waypoints in the simulation.

(7) As compared to backyard_flyer_solution.py, motion_planning.py has some changes to how grid-based A* planning is implemented. Those include:

- An additional state 'PLANNING' (line 22), that is initiated after arming the quadrotor (line 67) via the method 'plan_path' (line 114)
- When planning state is completed, the takeoff_transition is initiated (line 68)
- The 'plan_path' method (line 114) creates the configuration space representation, plans the path from start to goal, and defines the waypoints
- An additional 'send_waypoints' method (line 109) for sending waypoints to the simulator
- In the 'waypoint_transition' method (line 85), self.cmd_position now receives the heading direction parameter from self.target_position[3]
- The 'calculate_box' method for defining the quadratic waypoints in backyard_flyer_solution.py (line 72) has been removed from motion_planning.py
- In the 'arming_transition' method (motion_planning.py, line 74), no home position, and in 'takeoff_transition' method (motion_planning.py, line 80) no target altitude are specified anymore.
- MavLinkConnection arguments are provided by 'argparse.ArgumentParser' (line 175), and time.sleep is reduced to 1 second (line 182).



### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


