import numpy as np
import pandas as pd
import os
import traceback
import time
import sys
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

app = QApplication(sys.argv)

length = 0.1
Vmax = 0.001

a2 = 8*Vmax*Vmax/(3*length)
a1 = -np.sqrt((a2**3)/(6*length))

te = -a2/a1
x = np.arange(0, te, 0.001)
y = a1 * (x ** 2) + a2 * x

win = pg.GraphicsWindow(title="Data") # creates a window
p = win.addPlot(title="Realtime plot")  # creates empty space for the plot in the window
curve = p.plot()

ww = 500
Xm = np.linspace(0, 0, 500)
x = -ww
while True:
    Xm[:-1] = Xm[1:]
    y = a1 * (x ** 2) + a2 * x
    Xm[-1] = y  # vector containing the instantaneous values
    x += 5  # update x position for displaying the curve
    curve.setData(Xm)  # set the curve with this data
    curve.setPos(x, 0)  # set x position in the graph to 0
    QtGui.QApplication.processEvents()  # you MUST process the plot now
    time.sleep(0.05)

pg.QtGui.QApplication.exec_()
status = app.exec_()
sys.exit(status)






#
#
# import matplotlib.pyplot as plt
# import numpy as np
# from matplotlib.patches import Rectangle
#
# # creating space
# fig = plt.figure()
# ax = plt.subplot()
#
# #true value of integral
# integral = -4
#
# #segment start and finish points
# a = -2
# b = 0
#
# # segment length
# length = abs(b - a)
#
# # given function
# def f(x):
#     return x**3
#
# # function for drawing integral sums
# def draw_integral_sums(n, type):
#     s = 0
#     if type == 1:
#         for i in range(0, n):
#             xk = a+length*i/n
#             s += length/n * f(xk)
#             ax.add_patch(Rectangle((xk, 0), length/n, f(xk)))
#     if type == 2:
#         for i in range(0, n):
#             xk = a+length*i/n
#             s += length / n * f((2*xk + length/n) / 2)
#             ax.add_patch(Rectangle((xk, 0), length/n, f((2*xk + length/n) / 2)))
#     if type == 3:
#         for i in range(0, n):
#             xk = a+length*i/n
#             s += length / n * f(xk + length/n)
#             ax.add_patch(Rectangle((xk, 0), length/n, f(xk + length/n)))
#     print("Square: " + str(s))
#     print("Infelicity: " + str(abs(s - integral)))
#
# #data for function
# x_data = np.arange(a, b, 0.01)
# y_data = f(x_data)
#
# #drawing function
# ax.plot(x_data, y_data, color='orange')
# draw_integral_sums(100, 3)
#
# ax.legend(["Функция", "Интегральная сумма"])
# ax.set_ylabel("Y")
# ax.set_xlabel("X")
# #ax.plot([-2, 0], [0, 0], '--', color='black')
# #ax.plot([0, 0], [-8, 0], '--', color='black')
# ax.grid(which='major')
# ax.minorticks_on()
# ax.grid(which='minor',
#         color='black',
#         linestyle=':')
#
# plt.show()