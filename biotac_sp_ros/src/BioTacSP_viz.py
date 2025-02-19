#!/usr/bin/env python3
"""
Consider one BioTac SP sensor
1. Subscribe to /biotac_sp_pub_calib
2. Pop up windows to show
    24 ele as an image
    Pac as waveform
"""

# import roslib; roslib.load_manifest('biotac_logger')
import rospy
import os, sys
from types import *
import time
import numpy as np

from biotac_sp_ros.msg import BioTacSP

import cv2
from rospy.numpy_msg import numpy_msg
# from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32

# Set global variables
PATH = os.path.dirname(os.path.realpath(__file__))

# field_name dictionary to adress the respective sensors
fn = {
    "E1": 1, "PAC": 2, "E2": 3, "E3": 5, "E4": 7, "E5": 9, "E6": 11, "E7": 13, "E8": 15, \
    "E9": 17, "E10": 19, "E11": 21, "E12": 23, "E13": 25, "E14": 27, "E15": 29, \
    "E16": 31, "E17": 33, "E18": 35, "E19": 37, "E20": 39, "E21": 41, "E22": 43, \
    "E23": 45, "E24": 47, "PDC": 49, "TAC": 51, "TDC": 53
}
ele_idx = np.arange(0, 47, 2)
pac_idx = np.arange(1, 54, 2)
pdc_idx = 48
tac_idx = 50
tdc_idx = 52

class BioTacSPVisualizer:
    def __init__(self):
        rospy.init_node('BioTacSP_visualizer')
        self.sub = None
        self.pub = None


    def vizCallback(self, data):

        pdc_data = data.pdc_data
        pac_data = data.pac_data
        tdc_data = data.tdc_data
        tac_data = data.tac_data
        e = list(data.ele_data)

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
        #self.pub.publish(ele_arr.flatten('F'))

        aux=np.array(ele_arr,dtype=np.uint8)
        scale_percent = 4000
        width = int(aux.shape[1]*scale_percent/100)
        height = int(aux.shape[0]*scale_percent/100)
        dim = (width, height)
        #print("before resize aux {}, dim {}".format(aux.shape, dim)) #aux (11, 7), dim (280, 440)
        aux = cv2.resize(aux, dim, interpolation=cv2.INTER_AREA)
        im_color = (cv2.applyColorMap(aux, cv2.COLORMAP_HOT))
        #print(im_color.shape) # (440, 280, 3)
        #cv2.imshow("Sensor 0", im_color)
        cv2.imshow("Sensor 0", aux)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('Quit')
            cv2.destroyAllWindows()


    def listener(self):
        print("initiate viz listener")
        self.sub = rospy.Subscriber("/biotac_sp_pub_calib", BioTacSP, self.vizCallback, queue_size=1000)
        self.pub = rospy.Publisher("/biotac_sp_viz", numpy_msg(Floats), queue_size=10)
        rospy.spin()

if __name__ == '__main__':
    bt_viz = BioTacSPVisualizer()
    bt_viz.listener()
