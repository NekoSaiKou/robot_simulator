"""Simple scara straight line trajectory simulator"""

import arm
import cv2
import numpy as np
from math import hypot, pi, sin, cos

if __name__ == "__main__":
    robot = arm.Scara()
    robot.update()

    center = np.array([320,320])
    clockwise = 1
    cv2.namedWindow("test", cv2.WINDOW_NORMAL)
    counter = 0
    vel = 0
    while True:
        # Blank frame
        sim_frame = np.zeros((640,640,3))

        # Calculate Jacobian
        a0_s1   = robot.link_d[0] * sin(robot.theta_1)
        a0_c1   = robot.link_d[0] * cos(robot.theta_1)
        a1_s12  = robot.link_d[1] * sin(robot.theta_12)
        a1_c12  = robot.link_d[1] * cos(robot.theta_12)
        a2_s123 = robot.link_d[2] * sin(robot.theta_123)
        a2_c123 = robot.link_d[2] * cos(robot.theta_123)

        J = np.array([[-1*a0_s1-a1_s12-a2_s123, -1*a1_s12-a2_s123, -1*a2_s123],
                      [ 1*a0_c1+a1_c12+a2_c123,  1*a1_c12+a2_c123,  1*a2_c123],
                      [ 1                     ,  1                , 1]])

        # Velocity of end effector in this case is x:-500,y:0, theta: 0
        if counter % 600 == 0:
            vel = np.dot(np.linalg.inv(J),np.array([-500,0,0]))

        counter = counter + 1
        # Update angular velocity
        new_time = 2*pi/vel
        robot.set_velocity(0, new_time[0])
        robot.set_velocity(1, new_time[1])
        robot.set_velocity(2, new_time[2])

        # Update angle
        robot.update_angle(np.array([1 * clockwise, 1 * clockwise, 1 * clockwise]))

        # Update Robot
        robot.update()

        # Get command and Send to Robot
        keypos = robot.get_keypos()

        # Plot planar robot
        for i in range(1,len(keypos)):
            joint_f = (keypos[i-1][0:2]*[1, -1] + center).astype(int)
            joint_e = (keypos[i][0:2]*[1, -1] + center).astype(int)
            joint_f = tuple(joint_f)
            joint_e = tuple(joint_e)
            cv2.line(sim_frame, joint_f, joint_e, (0, 255, 0), 5)
            cv2.circle(sim_frame, joint_f, 10, (0, 0, 255), -1)
        cv2.imshow("test", sim_frame)
        key = cv2.waitKey(1)

        # q: quit, r: change direction
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('r'):
            clockwise = -1 * clockwise