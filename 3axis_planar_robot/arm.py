"""scara class"""
import json
import numpy as np
from math import sin, cos, pi

class Scara():
    """ Scara object """
    def __init__(self):
        """ Initialize robot current state """
        with open('config.json', 'r') as file:
            cfg_dict = json.load(fp=file)

            """
            Scara initial parameter. In this arm, only 2D is in the game. Z value is always 1.
            Beside, this is a three joint scara robot.
            """
            self.joint_a = np.array(cfg_dict["joint_a"]) * pi / 180.0
            self.joint_d = np.array(cfg_dict["joint_d"])
            self.link_a = np.array(cfg_dict["link_a"]) * pi / 180.0
            self.link_d = np.array(cfg_dict["link_d"])
            self.speed_a = np.array(cfg_dict["speed"])
            self.increment_a = 0.001 * 2 * pi/self.speed_a

    def __f_kinematics(self):
        """ Update transformation matrix """
        self.t01 = np.array([[cos(self.joint_a[0]), -1*sin(self.joint_a[0]), 0, cos(self.joint_a[0])*self.link_d[0]],
                             [sin(self.joint_a[0]),  1*cos(self.joint_a[0]), 0, sin(self.joint_a[0])*self.link_d[0]],
                             [0                   ,  0                     , 1, 0                                  ],
                             [0                   ,  0                     , 0, 1                                  ]])
        self.t12 = np.array([[cos(self.joint_a[1]), -1*sin(self.joint_a[1]), 0, cos(self.joint_a[1])*self.link_d[1]],
                             [sin(self.joint_a[1]),  1*cos(self.joint_a[1]), 0, sin(self.joint_a[1])*self.link_d[1]],
                             [0                   ,  0                     , 1, 0                                  ],
                             [0                   ,  0                     , 0, 1                                  ]])
        self.t23 = np.array([[cos(self.joint_a[2]), -1*sin(self.joint_a[2]), 0, cos(self.joint_a[2])*self.link_d[2]],
                             [sin(self.joint_a[2]),  1*cos(self.joint_a[2]), 0, sin(self.joint_a[2])*self.link_d[2]],
                             [0                   ,  0                     , 1, 0                                  ],
                             [0                   ,  0                     , 0, 1                                  ]])
        
        self.t02 = np.dot(self.t01, self.t12)
        self.t03 = np.dot(self.t02, self.t23)

        self.theta_123 = np.sum(self.joint_a[0:3])
        self.theta_12  = np.sum(self.joint_a[0:2])
        self.theta_1   = np.sum(self.joint_a[0:1])
        # self.t02 = np.array([[cos(theta_12), -1*sin(theta_12), 0, cos(theta_1)*self.link_d[0] + cos(theta_12)*self.link_d[1]],
        #                      [sin(theta_12),  1*cos(theta_12), 0, sin(theta_1)*self.link_d[0] + sin(theta_12)*self.link_d[1]],
        #                      [0            ,  0              , 1, 0                                  ],
        #                      [0            ,  0              , 0, 1                                  ]])

        # self.t03 = np.array([[cos(theta_123), -1*sin(theta_123), 0, cos(theta_1)*self.link_d[0] + cos(theta_12)*self.link_d[1] + cos(theta_123)*self.link_d[2]],
        #                      [sin(theta_123),  1*cos(theta_123), 0, sin(theta_1)*self.link_d[0] + sin(theta_12)*self.link_d[1] + sin(theta_123)*self.link_d[2]],
        #                      [0             ,  0               , 1, 0                                  ],
        #                      [0             ,  0               , 0, 1                                  ]])

        self.__update_jointpos(self.t01, self.t02, self.t03)

    def __update_jointpos(self, t01, t02, t03):
        """ Update keyposition data, called after __f_kinematics """
        self.keypos = []
        self.keypos.append(np.array([0,0,0]))
        self.keypos.append(t01[0:3, 3])
        self.keypos.append(t02[0:3, 3])
        self.keypos.append(t03[0:3, 3])
    
    def get_keypos(self):
        """ Get Scara joint positions in order to plot the arm """
        return self.keypos
    
    def set_velocity(self, id, time):
        """ Update velocity of given id """
        self.speed_a[id] = time
        self.increment_a[id] = 0.001 * 2 * pi/time

    def update_angle(self, direction):
        """ Update all motor angle """
        self.joint_a = self.joint_a + self.increment_a * direction

    def update(self):
        """ Update joint position and other parameter by forward kinematic """
        self.__f_kinematics()
