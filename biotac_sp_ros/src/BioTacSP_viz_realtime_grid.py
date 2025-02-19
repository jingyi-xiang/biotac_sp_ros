#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import rospy
import os, sys
from biotac_sp_ros.msg import BioTacSP

import numpy as np
from matplotlib.animation import FuncAnimation

TMAX = 200
pac_multiplier = 27

# TODO: frame rate is around 20Hz (update_counter is exactly 20, callback_counter slightly smaller)

class Visualiser:
    def __init__(self):
        self.callback_counter = 0
        self.update_counter = 0
        # create figure and gridspec
        self.fig = plt.figure()
        self.gs = self.fig.add_gridspec(4,7)
        self.ax_tdc = self.fig.add_subplot(self.gs[0,:4])
        self.ax_tac = self.fig.add_subplot(self.gs[1,:4])
        self.ax_pdc = self.fig.add_subplot(self.gs[2,:4])
        self.ax_pac = self.fig.add_subplot(self.gs[3,:4])
        self.ax_ele = self.fig.add_subplot(self.gs[:,4:])
        
        self.ax_tdc.set_title("Tdc")
        self.ax_tac.set_title("Tac")
        self.ax_pdc.set_title("Pdc")
        self.ax_pac.set_title("Pac")
        self.ax_ele.set_title("Ele")

        self.x_data, self.tdc_data, self.tac_data, self.pdc_data, self.pac_data = [], [], [], [], []
        self.x_data_pac = []
        self.ele_data = np.zeros((11,7))

        self.plt_tdc = self.ax_tdc.plot(self.x_data, self.tdc_data, c='r')[0]
        self.plt_tac = self.ax_tac.plot(self.x_data, self.tac_data, c='r')[0]
        self.plt_pdc = self.ax_pdc.plot(self.x_data, self.pdc_data, c='b')[0]
        self.plt_pac = self.ax_pac.plot(self.x_data_pac, self.pac_data, c='b')[0]
        self.plt_ele = self.ax_ele.imshow(self.ele_data, vmin=-20, vmax=200)


    def plot_init(self):
        for ax in [self.ax_tdc, self.ax_tac, self.ax_pdc, self.ax_pac]:
            ax.set_xlim(0, TMAX)
            ax.set_ylim(-10,200)


    def BT_callback(self, data):
        
        pdc_data = data.pdc_data
        pac_data = data.pac_data
        tdc_data = data.tdc_data
        tac_data = data.tac_data
        e = list(data.ele_data)
        x_index = len(self.x_data)

        self.x_data.append(x_index+1)
        self.tdc_data.append(tdc_data)
        self.tac_data.append(tac_data)
        self.pdc_data.append(pdc_data)

        self.x_data_pac.extend(list(np.linspace(x_index, x_index+1, pac_multiplier, endpoint=False)))
        self.pac_data.extend(pac_data)

        if len(self.x_data) > TMAX:
            current = self.x_data[-TMAX]
            self.x_data = [idx-current for idx in self.x_data[-TMAX:]]
            self.tdc_data = self.tdc_data[-TMAX:]
            self.tac_data = self.tac_data[-TMAX:]
            self.pdc_data = self.pdc_data[-TMAX:]
            self.pac_data = self.pac_data[-TMAX*pac_multiplier:]
            self.x_data_pac = [idx-current for idx in self.x_data_pac[-TMAX*pac_multiplier:]]
        
        # self.x_data_pac = list(np.linspace(min(self.x_data), max(self.x_data), len(self.x_data)*pac_multiplier, endpoint=False)) # somehow doesn't work
        self.ele_data = self.get_ele_arr(e)

        self.callback_counter += 1


    def update_plot(self, frame):
        self.plt_tdc.set_data(self.x_data, self.tdc_data)
        self.plt_tac.set_data(self.x_data, self.tac_data)
        self.plt_pdc.set_data(self.x_data, self.pdc_data)
        assert len(self.x_data_pac) == len(self.pac_data), "set_data x_data_pac {}, pac_data {}".format(len(self.x_data_pac), len(self.pac_data))
        self.plt_pac.set_data(self.x_data_pac, self.pac_data)
        self.plt_ele.set_data(self.ele_data)

        self.update_counter += 1
        #if self.update_counter % 100 == 0:
            #print("callback_counter {}, update_counter {}".format(self.callback_counter, self.update_counter)) # not exactly synchronous


    def get_ele_arr(self, e):
        ele_arr = np.array([[0, e[10], 0, 0, 0, e[0], 0],
                        [0, 0, e[11], 0, e[1], 0, 0],
                        [0, 0, 0, e[20], 0, 0, 0],
                        [0, 0, e[22], 0, e[21], 0, 0],
                        [e[12], e[13], 0, e[23], 0, e[3], e[2]],
                        [0, 0, e[14], 0, e[4], 0, 0],
                        [e[15], 0, 0, 0, 0, 0, e[5]],
                        [0, e[16], 0, 0, 0, e[6], 0],
                        [0, 0, e[17], 0, e[7], 0, 0],
                        [0, e[18], 0, 0, 0, e[8], 0],
                        [e[19], 0, 0, 0, 0, 0, e[9]]], dtype=np.float32)

        aux=np.array(ele_arr,dtype=np.uint8)
        return aux
        #scale_percent = 4000
        #width = int(aux.shape[1]*scale_percent/100)
        #height = int(aux.shape[0]*scale_percent/100)
        #dim = (width, height)
        ##print("before resize aux {}, dim {}".format(aux.shape, dim)) #aux (11, 7), dim (280, 440)
        #aux = cv2.resize(aux, dim, interpolation=cv2.INTER_AREA)
        #im_color = (cv2.applyColorMap(aux, cv2.COLORMAP_HOT))
        #return im_color

rospy.init_node('BioTacSP_viz_realtime')
vis = Visualiser()
sub = rospy.Subscriber("/biotac_sp_pub_calib", BioTacSP, vis.BT_callback, queue_size=100)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, interval=1)
plt.tight_layout()
plt.show(block=True)
