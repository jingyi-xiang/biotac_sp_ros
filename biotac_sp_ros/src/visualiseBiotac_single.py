#!/usr/bin/env python3
from __future__ import print_function

# -*- coding: utf-8 -*-
"""
:ABSTRACT:
This script is part of the of the enhanced grasp project

:REQUIRES:

:
:AUTHOR:  Pedro Machado
:ORGANIZATION: Nottingham Trent University
:CONTACT: pedro.baptistamachado@ntu.ac.uk
:SINCE: 10/04/2019
:VERSION: 0.1

2019 (c) GPLv3, Nottingham Trent University
Computational Neuroscience and Cognitive Robotics Laboratory
email:  pedro.baptistamachado@ntu.ac.uk
website: https://www.ntu.ac.uk/research/groups-and-centres/groups/computational-neuroscience-and-cognitive-robotics-laboratory


"""
# ===============================================================================
# PROGRAM METADATA
# ===============================================================================
__author__ = 'Pedro Machado'
__contact__ = 'pedro.baptistamachado@ntu.ac.uk'
__copyright__ = '2019 (C) GPLv3, CNCR@NTU, Prof. Martin McGinnity martin.mcginnity@ntu.ac.uk'
__license__ = 'GPLv3'
__date__ = '12/07/2019'
__version__ = '1.0'
__file_name__ = 'visualiseBiotac.py'
__description__ = 'Subscribe the Biotac sensors raw data and display the data per sensor'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "i386, x86_64, arm32 and arm64"
__diff__= "GPLv3 , new launch file and publication in 3 topics"

#===============================================================================
# IMPORT STATEMENTS
#===============================================================================
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import os
# from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg

#===============================================================================
# GLOBAL VARIABLES DECLARATIONS
#===============================================================================
PATH=os.path.dirname(os.path.realpath(__file__))
P=0.98
visualisationFlag = True# False #
global fsr
#===============================================================================
# METHODS
#===============================================================================
def callback_biotac_single(data,publisher):
    global flag, prev_mat, fsr
    buffer = data.data
    buffer = buffer.split(',')
    mat = np.asarray(buffer)

    mat = np.asarray(mat)

    # Dictionary to adress the respective sensors
    fn = {
        "E1": 1, "PAC": 2, "E2": 3, "E3": 5, "E4": 7, "E5": 9, "E6": 11, "E7": 13, "E8": 15, \
        "E9": 17, "E10": 19, "E11": 21, "E12": 23, "E13": 25, "E14": 27, "E15": 29, \
        "E16": 31, "E17": 33, "E18": 35, "E19": 37, "E20": 39, "E21": 41, "E22": 43, \
        "E23": 45, "E24": 47, "PDC": 49, "TAC": 51, "TDC": 53
    }



    if flag:
        prev_mat=mat[1:len(mat)]
        flag=False
    else:
        for i in range(1, len(mat)):
            prev_mat[i-1]=int((1-P)*float(mat[i])+P*float(prev_mat[i-1]))
        for i in range(1, len(mat)):
            mat[i] = np.abs(int(mat[i])-int(prev_mat[i - 1])) # RH: compute the change/derivative here, learn how it is done
    
    pac=0
    for i in range(2,len(fn)+1,2):
        pac+=int(mat[i])
    pac=int(pac/int(len(fn)))
    vis_mat=np.asarray([[0,int(mat[fn["E11"]]),0,0,0,int(mat[fn["E1"]]),0],
                        [0,0,int(mat[fn["E12"]]),0,int(mat[fn["E2"]]),0,0],
                        [int(mat[fn["TAC"]]),0,0,int(mat[fn["E21"]]),0,0,pac],
                        [0,0,int(mat[fn["E23"]]),0, int(mat[fn["E22"]]),0,0],
                        [int(mat[fn["E13"]]),int(mat[fn["E14"]]),0,int(mat[fn["E24"]]),0,int(mat[fn["E4"]]),int(mat[fn["E3"]])],
                        [0,0,int(mat[fn["E15"]]),0,int(mat[fn["E5"]]),0,0],
                        [int(mat[fn["E16"]]),0,0,0,0,0,int(mat[fn["E6"]])],
                        [0,int(mat[fn["E17"]]),0,0,0,int(mat[fn["E7"]]),0],
                        [int(mat[fn["TDC"]]),0,int(mat[fn["E18"]]),0,int(mat[fn["E8"]]),0,int(mat[fn["PDC"]])],
                        [0,int(mat[fn["E19"]]),0,0,0,int(mat[fn["E9"]]),0],
                        [int(mat[fn["E20"]]),0,0,0,0,0,int(mat[fn["E10"]])]])

    publisher.publish(np.asarray(vis_mat, dtype=np.float32).flatten('F'))

    aux=np.array(vis_mat,dtype=np.uint8)
    if visualisationFlag:
        scale_percent = 4000  # percent of original size
        width = int(aux.shape[1] * scale_percent / 100)
        height = int(aux.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        aux = cv2.resize(aux, dim, interpolation=cv2.INTER_AREA)
        im_color = (cv2.applyColorMap(aux, cv2.COLORMAP_HOT))
        cv2.imshow("Sensor 0", im_color)

    if visualisationFlag and cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('Quit')
        cv2.destroyAllWindows()


def listener():
    global flag
    while not rospy.is_shutdown():
        try:
            pub0 = rospy.Publisher('sensors/biotac/0', numpy_msg(Float32),queue_size=10)

            rospy.Subscriber("/biotac_sp_ros_single", String, callback_biotac_single, (pub0))
            print("Sensor 0 published in topic: /sensors/biotac/0.")
            flag=True
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down the Biotac subscriber!")
        except IOError:
            print(IOError)
            print("Shuting down the Biotac subscriber!")
#===============================================================================
#  TESTING AREA
#===============================================================================

#===============================================================================
# MAIN METHOD
#===============================================================================
if __name__ == '__main__':
    print("[Initialising biotac visualisation...]\n")
    rospy.init_node('visualise_biotac', anonymous=True)
    fsr=0
    listener()