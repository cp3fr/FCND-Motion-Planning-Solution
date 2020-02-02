import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import math

from planning_utils import a_star, heuristic, create_grid, path_pruning, grid_goal_verification
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

        # Compute current local position from drone latitude, longitude and altitude
        local_position = global_to_local((self._longitude, self._latitude, self._altitude), self.global_home)
        print('current local position: {}'.format(local_position))
        

        #append last waypoint (must be integers)
        # waypoints = []
        # waypoints.append([int(local_position[0]+ 1), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.   ])
        # waypoints.append([int(local_position[0]+ 2), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.1  ])
        # waypoints.append([int(local_position[0]+ 3), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.2  ])
        # waypoints.append([int(local_position[0]+ 4), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.3  ])
        # waypoints.append([int(local_position[0]+ 5), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.4  ])
        # waypoints.append([int(local_position[0]+ 6), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.5  ])
        # waypoints.append([int(local_position[0]+ 7), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.6  ])
        # waypoints.append([int(local_position[0]+ 8), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.7  ])
        # waypoints.append([int(local_position[0]+ 9), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.8  ])
        # waypoints.append([int(local_position[0]+10), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -0.9  ])
        # waypoints.append([int(local_position[0]+11), int(local_position[1]), TARGET_ALTITUDE, 3.14168 * -1.0  ])
        # waypoints.append([int(local_position[0]+12), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  1.1  ])
        # waypoints.append([int(local_position[0]+13), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  1.2  ])
        # waypoints.append([int(local_position[0]+14), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  1.3  ])
        # waypoints.append([int(local_position[0]+15), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  1.4  ])
        # waypoints.append([int(local_position[0]+16), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  1.5  ])
        # waypoints.append([int(local_position[0]+17), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  1.6  ])
        # waypoints.append([int(local_position[0]+18), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  1.7  ])
        # waypoints.append([int(local_position[0]+19), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  1.8  ])
        # waypoints.append([int(local_position[0]+20), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  1.9  ])
        # waypoints.append([int(local_position[0]+ 0), int(local_position[1]), TARGET_ALTITUDE, 3.14168 *  2.0  ])

        waypoints = [(25, -11, 5, -0.42029075143550804), (121, -16, 5, -0.05768878931449782)]
        print('waypoints plus heading:\n{}'.format(waypoints))
        print('waypoints:\n{}'.format(waypoints))
        print('type(waypoints) ',type(waypoints))
        print('type(waypoints[0][0]) ',type(waypoints[0][0]))
        print('type(waypoints[0][1]) ',type(waypoints[0][1]))
        print('type(waypoints[0][2]) ',type(waypoints[0][2]))
        print('type(waypoints[0][3]) ',type(waypoints[0][3]))

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
