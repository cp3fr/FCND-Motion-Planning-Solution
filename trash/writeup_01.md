3D Motion Planning Project

by Christian Pfeiffer

25.01.2020

===========================
1. Explain the starter code
===========================

Note: lines refer to lines in motion.planning.py

additional state Planning (line 22)

sate numbering is set to auto() (lines 16-22)

no more "self.all_waypoints = self.calculate_box()" during local_position_callback (line 45)

state_callback, during flight_state execute "self.plan_path()" instead of takeoff_transition (line 67)

additional callback state "States.PLANNING" that triggers "self.takeoff_transition" (line 69)

method "calculate_box" removed where local waypoints were defined (line 73)

swapped order self.arm() and self.take_control() (line 77-78)

no self.set_home_position and no manual setting of self.flight_state= States.ARMING (line 78)

in method "takeoff_transition", target position is specified by a different variable (from outside of the method), (line 83)

in method "waypoint_transition", the self.cmd_position, now includes additional parameter for the rotation angle (self.target_position[3]) (line 90)

new method "send_waypoints" (line 109ff), where waypoints are loaded via "msgpack.dumps(self.waypoints)" and then written to "self.connection._master.write(data)" (line 112)

new method "plan_path" (line 114ff), where target altitude and safety distance parameters are defined
obstacle map is loaded
grid is created
grid coordinates are extracted
a_star is used for planning a path between grid_start and grid_goal
waypoints are defined based on the path and saved to self.waypoints and method send_waypoints

start method uses self.connection.start() instead of super().start() (line 165)

The main method adds the port and host information via parsers, a timeout of 60 is defined,(line 176 ff)
sleep time is reduced to 1

planning is based on 2d grid (including safety distance)

planning_utils
uses six linear actions only and manhattan distance as heuristic
