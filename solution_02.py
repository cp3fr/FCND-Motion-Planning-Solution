import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

# from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global
from sampling import Sampler
from shapely.geometry import Polygon, Point, LineString
from graph_tools import create_graph, closest_point, a_star_graph, path_pruning, heuristic
import networkx as nx


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
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

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
        print('..starting plan_path method')
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # use latitude and longitude from colliders map as gobal home
        (lat0, lon0) = [ np.float64(s.split(' ')[1]) for s in np.loadtxt(
            'colliders.csv', delimiter='\n', dtype='str')[0].split(', ') ]
        self.set_home_position(lon0, lat0, 0.)

        print('global home {0}\nposition {1}\nlocal position {2}\n'.format(self.global_home, self.global_position, self.local_position))

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)


        # random sampler object, specify the flight altitude (range) 
        print('..initialize sampler, extract polygons')
        sampler = Sampler(data, TARGET_ALTITUDE)
        

        #get the polygons of the obstacles
        polygons = sampler._polygons
        # print(polygons)

        #randomly sample a certain number of points
        print('..sample points and check for collisions')
        num_samples = 100
        nodes = sampler.sample(num_samples)
        # print(nodes)

        #create the graph with a given maximal number of connections 
        print('..create graph')
        num_connections = 3
        g = create_graph(nodes, num_connections, polygons)

        #define start and goal positions
        print('..define start location')
        # ned_start = self.local_position
        ned_start = global_to_local(self.global_home, self.global_home)
        print('start (NED): {}'.format(ned_start))

        # lon =  -122.3978
        # lat =    37.7925
        # ned_goal = global_to_local((lon, lat, -TARGET_ALTITUDE), self.global_home)
        print('..define goal location')
        ned_goal = [ned_start[0]+30, 
                    ned_start[1]+30, 
                    ned_start[2]]
        print('goal (NED): {}'.format(ned_goal))



        #find closest nodes for start and goal
        print('..find nodes closest to start and goal location')
        start = closest_point(g, ned_start)
        goal = closest_point(g, ned_goal)
        print('start node (NED): {}'.format(start))
        print('goal node (NED): {}'.format(goal))


        path=[]
        waypoints=[]
        if start==goal:

            print('..start and goal are identical: no path is planned')

        else:

            #run a*
            print('..run astar graph version')
            path, _ = a_star_graph(g, heuristic, start, goal)
            print(len(path), path)


            print('..prune the path')
            path = path_pruning(path, epsilon=1e-3)

            # Convert path to waypoints
            waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]



        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
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
