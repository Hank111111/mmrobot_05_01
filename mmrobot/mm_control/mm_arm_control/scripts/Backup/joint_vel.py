#! /usr/bin/python

import rospy
from sensor_msgs.msg import JointState

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# global variable for vel.
vel_info = np.zeros(100)
vel_info_temp = vel_info

vel_temp = 0

# global variables for plot.
fig, ax = plt.subplots()
line, = ax.plot(vel_info)
ax.set_ylim(-6e-2, 6e-2)
ax.set_ylabel("joint vel: rad/s")
ax.set_xlabel("time")

def stateMsgReceived(state_msg):
    global vel_temp
    vel_temp = state_msg.velocity[15]
    print vel_temp

def update(frame):
    global vel_info
    global vel_info_temp
    global line

    vel_info[0:-1] = vel_info_temp[1:]
    vel_info[-1] = vel_temp
    vel_info_temp = vel_info

    line.set_ydata(vel_info)
    return line


def main():
    global vel_info
    global fig
    rospy.init_node("joint_vel_drawer")
    joint_sub = rospy.Subscriber("/robot/joint_states", JointState, stateMsgReceived)

    ani = animation.FuncAnimation(fig, update, frames=None, interval=50) 
    plt.show()


if __name__ == "__main__":
    main()
