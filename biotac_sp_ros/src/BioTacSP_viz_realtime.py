#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import rospy
import os, sys
from biotac_sp_ros.msg import BioTacSP

import numpy as np
from matplotlib.animation import FuncAnimation


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [])
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 1000)
        self.ax.set_ylim(-10, 200)
        return self.ln


    def BT_callback(self, data):

        pdc_data = data.pdc_data
        pac_data = data.pac_data
        tdc_data = data.tdc_data
        tac_data = data.tac_data
        e = list(data.ele_data)
        x_index = len(self.x_data)
        #print("pdc", pdc_data)
        #print("x {}, y{}".format(self.x_data, self.y_data))
        self.x_data.append(x_index+1)
        self.y_data.append(pdc_data)


    def update_plot(self, frame):
        print("update plot", self.x_data, self.y_data)
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln


rospy.init_node('BioTacSP_viz_realtime')
vis = Visualiser()
sub = rospy.Subscriber("/biotac_sp_pub_calib", BioTacSP, vis.BT_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True)
