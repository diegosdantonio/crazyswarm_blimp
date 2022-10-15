#!/usr/bin/env python

import rospy
import numpy as np
from pycrazyswarm import *
import uav_trajectory

from sensor_msgs.msg import Joy


lastData = None


def joyChanged(data):
    global lastData
    lastData = data


Z = 0.5




if __name__ == "__main__":

    swarm = Crazyswarm()
    #timeHelper = swarm.timeHelper

    cf = swarm.allcfs.crazyflies[0]
    #cf.loc.send_ext_pose(np.array([0,0,0]),np.array([0,0,0,1]))
    timeHelper = swarm.timeHelper

    cf.takeoff(targetHeight=0.5, duration=Z+1.0)
    timeHelper.sleep(1.5+Z)

    e = uav_trajectory.TrajectoryOutput()

    rate = 30.0

    joy_topic = "/joy"
    # rospy.Subscriber(joy_topic, Joy)
    rospy.Subscriber(joy_topic, Joy, joyChanged)
    zp=0
    
    while not swarm.input.checkIfButtonIsPressed():

        if lastData != None:
            # print(lastData.axes[0])
            
            yaw=lastData.axes[0]
            xp=lastData.axes[4]*10
            yp=lastData.axes[3]*10
            zp+=lastData.axes[1]

        else:
            yaw=0
            xp=0
            yp=0
            zp=0

        print(xp, yp, zp)

        # for cf in allcfs.crazyflies:
            # pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
            # cf.goTo(pos, 0, 1.0)

        e.pos = np.array([0,0,0.])
        e.vel = np.array([xp, yp, zp])
        e.acc = np.array([0,0,0.])
        e.yaw = 0
        e.omega = np.array([0,0,0.])

        # cf.cmdFullState(
        # e.pos,
        # e.vel,
        # e.acc,
        # e.yaw,
        # e.omega)

        cf.cmdVelocityWorld(np.array([xp, yp, zp]), yawRate=0)
        # cf.cmdVel(xp, yp, 0., zp)
        #cf.cmdVel(40000, 40000, 40000, 40000)
        # timeHelper.sleep(0.1)
        timeHelper.sleepForRate(rate)

    # print("press button to continue...")
    # swarm.input.waitUntilButtonPressed()

    cf.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)