import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import math

from receding_horizon_utils import *
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

        self.voxmap = np.zeros((40, 40, 40), dtype=np.bool)
        self.sampler = None
        self.past_time = time.time()
        self.current_time = self.past_time

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


            #check for collisions based on local position update and a fixed minimum time (1 sec)
            self.current_time = time.time()

            if self.current_time - self.past_time > 1.0:

                self.past_time = self.current_time
                print(self.current_time)

                #check for obstacle in path within receding horizon
                if self.local_obstacle():
                    print('..obstacle detected, updating waypoints')
                    #if obstacle detected
                    self.update_waypoints()
                else:
                    print('..no obstacle')


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

        # Use more than 5m safety distance to work well with the simulator
        SAFETY_DISTANCE = 7

        # Set target altitude
        TARGET_ALTITUDE = 5

        # Set target position (really needed?)
        self.target_position[2] = TARGET_ALTITUDE

        # Read home latitude and longitude from colliders.csv
        (lat0, lon0) = [ np.float64(s.split(' ')[1]) for s in 
            np.loadtxt('colliders.csv', delimiter='\n', dtype='str')[0].split(', ') ]
       
        # Setting home position
        print('..setting gobal home to {} latitude, {} longitude, {} altitude'.format(lat0, lon0, 0.))
        self.set_home_position(lon0, lat0, 0.)
        
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        print('..extract polygons')
        sampler = Sampler(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        self.sampler = sampler


        ## quick hack: simple path with collision

        local_start = self.local_position
        local_goal = global_to_local((-122.397882, 37.794821,  -TARGET_ALTITUDE), self.global_home)
        if np.linalg.norm(local_start-local_goal) < 10:
            local_goal = global_to_local((lon0, lat0, -TARGET_ALTITUDE), self.global_home)

        print(local_start)
        print(local_goal)

        waypoints = [[int(local_start[0]), int(local_start[1]), int(local_start[2]), 0], [int(local_goal[0]), int(local_goal[1]), int(local_goal[2]), 0]]
        print('waypoints: ',waypoints)



        ## original probabilistic roadmap stuff

        # num_samples = 150
        # print('..sample {} points and check for collisions'.format(num_samples))
        # nodes = sampler.sample(num_samples)
        # print('....sampled {} points in free space'.format(len(nodes)))

        # num_connections = 15
        # print('..create graph with up to {} edges per node'.format(num_connections))
        # g = create_graph(nodes, num_connections, sampler)
        # print('....graph has {} nodes and {} edges'.format(len(g.nodes), len(g.edges)))

        # # Use local position as start location
        # print('..setting start and goal location')
        # start = self.local_position
        # print('....start: {}'.format(start))

        # # Set some goal location by longitude and latitude
        # # (lat, lon) =(lat0, lon0) #home
        # # (lat, lon) =(37.794821, -122.397882) #FEDEX
        # # (lat, lon) =(37.794598, -122.396599) #Hyatt
        # # (lat, lon) =(37.793373, -122.398809) #California Street
        # (lat, lon) =(37.793685, -122.396311) #Embarcadero Station
        # # (lat, lon) =(37.793171, -122.396678) #Peet's Coffee
        # # (lat, lon) =(37.796176, -122.398189) #somewhere north
        # # (lat, lon) =(37.791822, -122.394929) #somewhere south east
        # # (lat, lon) =(37.793448, -122.398147)
        # # (lat, lon) =(37.793614, -122.396895)
        # # (lat, lon) =(37.792575, -122.397441)
        # # (lat, lon) =(37.793448, -122.398147)
        # # (lat, lon) =(37.793982, -122.397718)
        # # (lat, lon) =(37.793614, -122.396895)

        # # Convert goal location from geodetic to grid frame
        # goal = global_to_local((lon, lat, -TARGET_ALTITUDE), self.global_home)
        # print('....goal:  {}'.format(goal))

        # #find closest nodes for start and goal
        # start = closest_point(g, start)
        # goal = closest_point(g, goal)
        # print('....start node: {}'.format(start))
        # print('....goal node:  {}'.format(goal))

        # #run a*
        # print('..run a* on graph')
        # path, _ = a_star_graph(g, heuristic, start, goal)
        # if len(path)>0:
        #     path.append(goal)
        # print('....path found by a*: {}\n'.format(path))

        # # Prune path to reduce the number of waypoints
        # path = path_pruning(path, epsilon=0.1)
        # print('....path after pruning:\n{}'.format(path))

        # # Convert path to waypoints
        # waypoints = waypoints_from_path(path, self.local_position, heading=True)
        # print('....waypoints:\n{}'.format(waypoints))

        # Send waypoints to sim (this is just for visualization of waypoints)
        self.waypoints = waypoints

        self.send_waypoints()

    def local_obstacle(self):
        #update voxmap accoring to current position
        self.voxmap = update_voxmap(self.voxmap, self.local_position, self.sampler)
        # m = self.voxmap[:,:,20]
        # for row in m:
        #     print(''.join([str(int(val)) for val in row]))
        #check that there are no collisions in the current path
        return check_obstacle(self.voxmap, self.target_position[:3] - self.local_position)



    def update_waypoints(self):
        print('')


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
