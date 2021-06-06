#!/usr/bin/env python3

import numpy as np
import copy
import math

debug = True

robot_dimention = {'rigid': {'radius': 10
                             },
                   'turtlebot': {'radius': 5,
                                 'len_between_wheel': 5,
                                 'radius_wheel': 1,
                                 'rpm': [10.0, 20.0]}
                   }

class robot:
    def __init__(self, state=None):
        self.state = None
        if isinstance(state, np.ndarray):
            self.state = np.copy(state)
        elif isinstance(state, list):
            self.state = np.array(state)

    def get_loc(self):
        return tuple((self.state[0:2]).astype(int))

    def get_state(self):
        return np.copy(self.state)

    def teleport(self, state):
        if type(state) == np.ndarray:
            assert state.shape == (2,) or state.shape == (3,), state
        if type(state) == list or type(state) == tuple:
            assert len(state) == 2 or len(state) == 3, state

        if isinstance(state, np.ndarray):
            self.state = np.copy(state)
        elif isinstance(state, list):
            self.state = np.array(state)
        elif isinstance(state, tuple):
            self.state = np.array(state)
        else:
            raise AssertionError('cannot recognize input state', state)
        return self

    def actionset(self):
        return [point_robot.moveUpRight,
                point_robot.move_low_right,
                point_robot.move_up_left,
                point_robot.move_low_left,
                point_robot.move_up, point_robot.move_right,
                point_robot.move_down, point_robot.move_left]

    def move_left(self):
        self.state[1] -= 1
        return 1

    def move_right(self):
        self.state[1] += 1
        return 1

    def move_up(self):
        self.state[0] += 1
        return 1

    def move_down(self):
        self.state[0] -= 1
        return 1

    def move_low_left(self):
        self.move_left()
        self.move_down()
        return math.sqrt(2)

    def move_low_right(self):
        self.move_right()
        self.move_down()
        return math.sqrt(2)

    def moveUpRight(self):
        self.move_up()
        self.move_right()
        return math.sqrt(2)

    def move_up_left(self):
        self.move_up()
        self.move_left()
        return math.sqrt(2)

    def copy(self):
        return robot(copy.deepcopy(self.state))

    def __str__(self):
        return str(self.get_loc())[1:-1]

    def next_moves(self, state, space):
        """
        all possible location next move can go
        :param state: current state of robot
        :type state: numpy.ndarray
        :return: a list of possible location of next move
        :rtype: list
        """
        states_next = []
        dises_next = []
        for x in np.arange(-1, 2):      # can only move 1 at a time
            for y in np.arange(-1, 2):  # can only move 1 at a time
                dis = math.sqrt(x**2 + y**2)
                move = np.array([x, y])
                state_copy = np.copy(state) + move
                if space.invalidArea(self.teleport(state_copy)):    # if robot is ok at this state in the space
                    states_next.append(state_copy)
                    dises_next.append(dis)
        return states_next, dises_next


class point_robot(robot):
    def __init__(self, state=None):
        super().__init__(state)

    def copy(self): return point_robot(copy.deepcopy(self.state))


class rigid_robot(robot):
    def __init__(self, state=None,
                 radius=robot_dimention['rigid']['radius'], step_resolution=1,
                 step_min=0.0, step_max=10.0,
                 angle_turn_min=0.0, angle_turn_max=np.pi, angel_turn_resolution=np.pi/3):
        """
        a rigid robot with radius
        :param state: robot state [x, y, angle]
        :type state:
        :param radius:
        :type radius:
        :param step_min:
        :type step_min:
        :param step_max:
        :type step_max:
        :param angel_turn_resolution: int or float
        :type angel_turn_resolution:
        """
        super().__init__(state)
        self.radius = radius
        self.step_resolution = step_resolution
        self.step_min, self.step_max = step_min, step_max
        self.angle_turn_min, self.angle_turn_max = angle_turn_min, angle_turn_max
        self.angel_turn_resolution = angel_turn_resolution  # change angle resolution from degree to radians

    def get_radius(self): return self.radius

    def copy(self): return rigid_robot(state=copy.deepcopy(self.state), radius=self.radius, step_resolution=self.step_resolution, step_min=self.step_min, step_max=self.step_max, angle_turn_min=self.angle_turn_min, angle_turn_max=self.angle_turn_max, angel_turn_resolution=self.angel_turn_resolution/math.pi*180)

    def next_moves(self, state_cur, space):
        """
        all possible location next move can go
        :param state_cur: current state of robot
        :type state_cur: numpy.ndarray
        :param space: physical space of the robot
        :return: a list of possible location of next move
        :rtype: list
        """
        state_next = []
        dises_next = []

        for turn in np.arange(-self.angle_turn_max, self.angle_turn_max+self.angel_turn_resolution, self.angel_turn_resolution):
            for step in np.arange(self.step_max, self.step_min, -self.step_resolution):
                self.teleport(state_cur)
                self.move((step, turn))
                if space.invalidArea(self):    # if robot is ok at this state in the space
                    state_next.append(self.get_state())
                    dises_next.append(0.5 * step)
        return state_next, dises_next

    def move(self, step):
        """
        move rigid robot
        :param step: step length
        :type step: int or float
        :return:
        :rtype:
        """
        self.state[-1] += step[-1]  # turn robot angle first
        self.state[0:-1] = self.state[0:-1] + step * np.array([np.cos(self.state[-1]), np.sin(self.state[-1])])


class turtlebot(robot):
    def __init__(self, state=None,
                 dis_between_wheel=robot_dimention['turtlebot']['len_between_wheel'],
                 radius_robot=robot_dimention['turtlebot']['radius'],
                 radius_wheel=robot_dimention['turtlebot']['radius_wheel'],
                 rpms=robot_dimention['turtlebot']['rpm']):
        super().__init__(state)
        self.dis_between_wheel = dis_between_wheel
        self.radius = radius_robot
        self.radius_wheel = radius_wheel
        self.rpms = rpms
        self.vs = [2*np.pi*radius_wheel * rpm / 60.0 for rpm in rpms]   # convert epm to meters per second

    def get_radius(self): return self.radius

    def get_wheel_radius(self): return self.radius_wheel

    def get_dis_wheel(self): return self.dis_between_wheel

    def next_moves(self, state_cur, time_step=0.1, space=None):
        """
        all possible location next move can go
        :param state_cur: current state of robot
        :type state_cur: numpy.array
        :param time_step: time resolution for this motion
        :return: a list of possible location of next move
        :rtype: list
        """
        states_next = []
        dises_next = []
        rpms_next = []
        vs_next = []

        theta = state_cur[-1]
        for v_left, rpm_left in zip(self.vs, self.rpms):
            for v_right, rpm_right in zip(self.vs, self.rpms):
                v = v_left, v_right

                self.teleport(state_cur)
                dis, w = self.move(v, time_step)

                if space.invalidArea(self):
                    # append next state
                    states_next.append(self.get_state())
                    dises_next.append(dis)
                    rpms_next.append({
                        'left': rpm_left,
                        'right': rpm_right
                    })
                    vs_next.append({
                        'vel':  float(dis) / time_step,
                        'angle': w
                    })

        return states_next, dises_next, rpms_next, vs_next

    def move(self, step, time_step=0.01):
        """
        
        :param step: 
        :param time_step: 
        :return: 
        """
        x, y, theta = self.state
        v_left, v_right = step
        # convert to velocity in global coord system
        v_x, v_y = self.radius / 2 * (v_left + v_right) * np.cos(theta), self.radius / 2 * (v_left + v_right) * np.sin(
            theta)
        w = self.radius / self.dis_between_wheel * (v_right - v_left)
        dis = np.sqrt(math.pow((v_x * time_step), 2) + math.pow((v_y * time_step), 2))
        d_x, d_y, d_theta = v_x * time_step, v_y * time_step, w * time_step

        self.state[-1] += d_theta  # turn robot angle first
        self.state[0:-1] = self.state[0:-1] + np.array([d_x, d_y])
        return dis, w
