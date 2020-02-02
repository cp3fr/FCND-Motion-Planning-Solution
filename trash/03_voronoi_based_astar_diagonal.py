import argparse
import time
import msgpack
from enum import Enum, auto
import networkx as nx

import numpy as np
import math

from planning_utils import *
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(
            self.target_position[0], 
            self.target_position[1], 
            self.target_position[2], 
            self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING

        print("Searching for a path ...")

        # Use more than 5m safety distance to work well with the simulator
        SAFETY_DISTANCE = 7

        # Set target altitude
        TARGET_ALTITUDE = 5

        # Set target position (really needed?)
        self.target_position[2] = TARGET_ALTITUDE

        # Read home latitude and longitude from colliders.csv
        (lat0, lon0) = [ np.float64(s.split(' ')[1]) for s in 
            np.loadtxt('colliders.csv', delimiter='\n', dtype='str')[0].split(', ') ]
        print('gobal home latitude = {} and longitude = {}'.format(lat0, lon0))
        
        # Set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.)

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        print('..getting voronoi edges')
        _, edges = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        print('..making the graph')
        g = nx.Graph()
        for e in edges:
            e = np.array(e).astype(int)
            dist = np.sqrt((e[0][0] - e[1][0])**2 + (e[0][1] - e[1][1])**2)
            g.add_edge(tuple(e[0]), tuple(e[1]), weight=dist)


        # Use local position as start location
        grid_start = self.local_position
        print('start: {}'.format(grid_start))

        # Set goal location by longitude and latitude
        # (lat, lon) =(37.793448, -122.398147)
        (lat, lon) =(37.793614, -122.396895)6


        # Convert goal location from geodetic to grid frame
        local_goal = global_to_local((lon, lat, -TARGET_ALTITUDE), self.global_home)
        grid_goal = local_goal
        print('goal:  {}'.format(grid_goal))

        #find closest nodes for start and goal
        print('..find nodes closest to start and goal location')
        nodes = list(g.nodes())

        idx = np.array([ np.linalg.norm(np.array(grid_start[:2])-np.array(n)) for n in nodes ]).argmin() 
        start = nodes[idx]

        idx = np.array([ np.linalg.norm(np.array(grid_goal[:2])-np.array(n)) for n in nodes ]).argmin() 
        goal = nodes[idx]

        print('start node: {}'.format(start))
        print('goal node:  {}'.format(goal))

        #run a*
        print('..run astar graph version')
        path, _ = a_star_graph(g, heuristic, start, goal)
        print('path before pruning: {}\n'.format(path))

        # Prune path to reduce the number of waypoints
        path = path_pruning(path, epsilon=0.1)
        print('path after pruning:\n{}'.format(path))

        # Convert path to waypoints
        waypoints = waypoints_from_path_and_altitude(path, TARGET_ALTITUDE, self.local_position, 0., 0., heading=True)
        print('waypoints:\n{}'.format(waypoints))

        # Set self.waypoints
        self.waypoints = waypoints
        
        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
