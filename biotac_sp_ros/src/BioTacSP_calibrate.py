#!/usr/bin/env python3
"""
Consider one BioTac SP sensor
1. Subscribe to /biotac_sp_ros_single)
2. Calibrate by computing the accummulated average.
    calibration can be triggerd by ```rostopic pub -1 /trigger_biotac_sp_calib std_msgs/String "calibrate"```
3. Publish the calibrated value as /biotac_sp_pub_calib
"""

# import roslib; roslib.load_manifest('biotac_logger')
import rospy
import os, sys
from types import *
import time
import numpy as np

from std_msgs.msg import String
from biotac_sp_ros.msg import BioTacSP

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

class BioTacSPListener:
    def __init__(self, num_pac=27, num_ele=24):
        rospy.init_node('BioTacSP_listener')
        self.init_sub = None
        self.data_sub = None
        self.pub = None
        self.calibrated = False
        self.pdc_mu = -1.0
        self.pac_mu = np.zeros(num_pac)
        self.tac_mu = -1.0
        self.tdc_mu = -1.0
        self.electrode_mu = np.zeros(num_ele)

        # temp_arrays
        self.calibration_duration = 200
        self.cal_count = 0
        self.pdc_array = np.zeros(self.calibration_duration)
        self.pac_array = np.zeros([self.calibration_duration, num_pac])
        self.tdc_array = np.zeros(self.calibration_duration)
        self.tac_array = np.zeros(self.calibration_duration)
        self.electrode_array = np.zeros([self.calibration_duration, num_ele])


    def biotacCallback(self, data):
        # TODO: why absolute values
        msg = BioTacSP()
        # parse string data
        buffer = data.data
        buffer = buffer.split(',')
        mat = np.asarray([int(x) for x in buffer[1:]])
        #print("mat {} {} {}".format(type(mat), type(mat[0]), mat.shape))

        if self.calibrated is True:
            msg.pdc_data = int(abs(mat[pdc_idx] - self.pdc_mu))
            msg.pac_data = np.abs(mat[pac_idx] - self.pac_mu).astype(int)
            msg.tdc_data = int(abs(mat[tdc_idx] - self.tdc_mu))
            msg.tac_data = int(abs(mat[tac_idx] - self.tac_mu))
            msg.ele_data = np.abs(mat[ele_idx] - self.electrode_mu).astype(int)
            self.pub.publish(msg)
        else:
            self.pdc_array[self.cal_count] = mat[pdc_idx]
            self.pac_array[self.cal_count, :] = mat[pac_idx]
            self.tdc_array[self.cal_count] = mat[tdc_idx]
            self.tac_array[self.cal_count] = mat[tac_idx]
            self.electrode_array[self.cal_count, :] = mat[ele_idx]
            self.cal_count += 1

            if self.cal_count == self.calibration_duration:
                self.pdc_mu = np.mean(self.pdc_array)
                self.pac_mu = np.mean(self.pac_array, axis=0)
                self.tdc_mu = np.mean(self.tdc_array)
                self.tac_mu = np.mean(self.tac_array)
                self.electrode_mu = np.mean(self.electrode_array, axis=0)
                print('Overall calibration status: ')
                print('pdc: ', self.pdc_mu, np.std(self.pdc_array))
                print('tdc: ', self.tdc_mu, np.std(self.tdc_array))
                print('tac: ', self.tac_mu, np.std(self.tac_array))
                self.calibrated = True
                print('end of calibration')


    def InitCallback(self, data):
        if data.data == 'calibrate':
            print("starting calibrate")
            self.calibrated = 0
            self.cal_count = 0

    def listener(self):
        self.init_sub = rospy.Subscriber("/trigger_biotac_sp_calib", String, self.InitCallback, queue_size=1000)
        self.data_sub = rospy.Subscriber("/biotac_sp_ros_single", String, self.biotacCallback, queue_size=1000)
        self.pub = rospy.Publisher("/biotac_sp_pub_calib", BioTacSP, queue_size=10)
        rospy.spin()

if __name__ == '__main__':
    bt_listener = BioTacSPListener()
    bt_listener.listener()