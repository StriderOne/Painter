import numpy as np
import pandas as pd
import os
import traceback
import time
import median_filter as md

import rtde_control
import URDriver

import sys
from PyQt5.QtWidgets import QApplication
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pt

# Setup robot
ip = '192.168.88.6'
rtde_c = rtde_control.RTDEControlInterface(ip)

DIR_F   = np.array([0, 0, -0.1, 0, 0, 0])*0.01
NUM_JOINTS = 6
# Joint movement
home_posistion = np.array([0, -90, -90, 0, 90, 0]) * np.pi / 180
rtde_c.moveJ(home_posistion)

# Cartesian space movements
up_height = 0.01

canvas_position = np.array([0.28, -0.27, 0.19691]) + np.array([0.225, 0.225, 0])

orient = np.array([2.222, -2.223, 0])
position = np.array([0.4922, -0.1333, 0.2342])
pose = np.concatenate((position, orient))

# Movement configuration
speed = 1
acceleration = 0.5
blend = 0

#rtde_c.moveL(pose, speed, acceleration)

# variables for PID control
last_err = 0
integral = 0
integral_time = 0

dt = 1.0/500
robot1 = URDriver.UniversalRobot(ip)

current_filepath = os.path.dirname(os.path.abspath(__file__))

median = md.MedianFilter(NUM_JOINTS, 5)

# path's length in meters
length = 0.1
Vmax = 0.001
a2 = 8*Vmax*Vmax/(3*length)
a1 = -np.sqrt((a2**3)/(6*length))

STAGE = 0

app = QApplication(sys.argv)
force = np.array([])
time_array = np.array([])
TIME_TOUCH = 0


win = pt.GraphicsWindow(title="Data") # creates a window
p = win.addPlot(title="Realtime plot")  # creates empty space for the plot in the window
curve = p.plot()

ww = 500
Xm = np.linspace(0, 0, 500)
ptr = -ww

def update_graph (force):
    ww = 500
    Xm = np.linspace(0, 0, 500)
    ptr = -ww
    while True:
        Xm[:-1] = Xm[1:]
        Xm[-1] = force  # vector containing the instantaneous values
        ptr += 1  # update x position for displaying the curve
        curve.setData(Xm)  # set the curve with this data
        curve.setPos(ptr, 0)  # set x position in the graph to 0
        QtGui.QApplication.processEvents()  # you MUST process the plot now
        time.sleep(0.05)


def find_object_force_control(force: np.ndarray, z_force) -> np.ndarray:
    global STAGE
    global TIME_TOUCH
    global last_err
    global integral
    global integral_time

    # Coefficients
    max_speed = 0.001

    vel = np.zeros(6)
    k_p = -0.0001
    k_i = -0.00001
    k_d = -0.00001

    # time
    i_new_time = time.time()
    delta_t = i_new_time - integral_time
    integral_time = i_new_time

    f_6 = force[:3]

    data = np.zeros(6)
    data[:3] = force[:3]

    # error
    goal = z_force
    err_f = goal - f_6[2]
    derr_f = (err_f - last_err)/delta_t
    last_err = err_f

    #drawing graph
    update_graph(f_6[2])

    # PID
    sum_f = k_p*err_f
    sum_f = sum_f if abs(sum_f) < max_speed else max_speed * np.sign(float(sum_f))
    sum_d = k_d*derr_f
    integral += k_i* delta_t * err_f

    dirv = -DIR_F if abs(f_6[2]) < 10 else DIR_F*0

    f_6[0] = 0
    f_6[1] = 0
    f_6[2] = sum_f + sum_d + dirv[2]
    vel[:3] = f_6
    # print('err', err_f, derr_f)

    if np.abs(err_f) < 1:
        STAGE = 1

    return vel




def make_trajectory():

    global STAGE
    global TIME_TOUCH

    count = 0

    # Move robot down until selected force

    while True:
        start = time.time()
        robot1.update_state()

        # Receive current forces
        fe = np.array(median.apply_median(robot1.state.f)).flatten()

        # Calculating speed to reach selected force
        speed = find_object_force_control(fe, 5)
        if (STAGE == 1):
            t = count * dt
            vx = a1 * t ** 2 + a2 * t
            if (vx <= 0):
                STAGE = 2
                robot1.control.servoStop()
                break
            speed[0] = vx
            count += 1


        # Move manipulator
        robot1.control.servoL(speed, acceleration, dt)

        end = time.time()
        duration = end - start
        if duration < dt:
            time.sleep(dt - duration)






# Main part

# make_trajectory()

# rtde_c.moveL(pose, speed, acceleration)
while True:
    fe = np.array(median.apply_median(robot1.state.f)).flatten()
    speed = find_object_force_control(fe, 5)


pt.QtGui.QApplication.exec_()
status = app.exec_()
sys.exit(status)

