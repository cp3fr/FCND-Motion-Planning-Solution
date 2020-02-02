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

        #global obstacle information (2.5D polygons)
        self.sampler = None 

        #local planner variables
        self.voxmap = np.zeros((40, 40), dtype=np.bool) #2D configuration space (1m cell size)
        self.t1 = time.time()     # current time in sec
        self.t0 = self.t1         # past time in sec
        self.sr_local_planner = 2 # sampling rate in Hz

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

            ## check if local waypoint has been reached

            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

            ## local planning: 

            #update current time
            self.t1 = time.time() 

            #if local planning update is due
            if self.t1 - self.t0 > 1 / self.sr_local_planner:

                #set past time to current time
                self.t0 = self.t1

                #perform local planning
                self.local_path_planning()




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


    def local_path_planning(self):

        #update local map
        self.voxmap = update_local_map(self.voxmap, self.local_position, self.sampler)

        #check for obstacles in local path
        (in_collision, local_path, collision_point) = local_path_obstacle_detection(self.voxmap, self.target_position[:3] - self.local_position)

        #print the local voxmap
        print_local_map(self.voxmap, local_path)

        #if local path in collision
        if in_collision:
            print('..collision detected: replan local path')
            print('collision_point ',collision_point)

            #find free space waypoint close to collision_point
            w=0 #search radius around the collision point
            d=np.linalg.norm(np.array(collision_point)) #distance of collision waypoint from local position in voxmap space

            #continue until no collision point is found or the maximum search radius has been reached
            while in_collision and w<np.max(self.voxmap.shape):
                
                #increase search radius
                w+=1

                #loop over points in search radius
                for i in range(collision_point[0]-w, collision_point[0]+w+1):

                    for j in range(collision_point[1]-w, collision_point[1]+w+1):

                        #check if point is valid
                        if i>=0 and i<=self.voxmap.shape[0] and j>=0 and j<=self.voxmap.shape[1]:

                            #if a free space is found
                            if self.voxmap[i,j]==False:

                                #if distance to free point is at least at distance of the collision point to the current location
                                if np.linalg.norm(np.array([i,j]))>d and np.linalg.norm(np.array([i,j]))<=d+2:

                                    print('free point close to collision ',[i,j])
                                    #make new waypoint
                                    new_waypoint = [
                                    int(self.local_position[0] - int(self.voxmap.shape[0]/2) + i),  
                                    int(self.local_position[1] - int(self.voxmap.shape[1]/2) + j), 
                                    int(-self.local_position[2]), 
                                    self.target_position[3]]

                 
                                    
                                    print('waypoints (before)', self.waypoints)
                                    print('target_position (before)', self.target_position)

                                    print('new waypoint: ',new_waypoint)
                                    self.waypoints[:0] = [self.target_position]
                                    self.waypoints[:0] = [new_waypoint]
        
                                    
                                    print('waypoints (after append)', self.waypoints)
                                    self.waypoint_transition()
                                    print('waypoints (after transition)', self.waypoints)


                                    print('target_position (after)', self.target_position)
                                    


                                    #set no collision 
                                    in_collision = False
                                    break








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
