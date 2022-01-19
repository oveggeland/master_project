#!/usr/bin/env python3

import rospy
from synchronizer.srv import *
import sys
import math
from time import time
from sensor_msgs.msg import Imu

try:
    from ros_openimu.src.aceinna.tools import OpenIMU
except:  # pylint: disable=bare-except
    temp = (sys.path[0])
    temp2 = temp[0:(len(temp)-7)]
    sys.path.append(temp2 + 'src')
    #sys.path.append('./src')
    from aceinna.tools import OpenIMU


class OpenIMUros:
    def __init__(self):
        self.openimudev = OpenIMU()
        self.openimudev.startup()

    def close(self):
        self.openimudev.close()

    '''
    def readimu(self, packet_type):
        readback = self.openimudev.getdata(packet_type)
        return readback
    '''

    def readimu(self):
        readback = self.openimudev.getdata('o1')
        return readback

if __name__ == "__main__":
    rospy.init_node("openimu_driver")

    pub_imu = rospy.Publisher('imu_acc_ar', Imu, queue_size=5)
    imu_msg = Imu()             # IMU data
        
    rate = 200  # 200Hz
    clk = rospy.Rate(rate)   
    seq = 0
    frame_id = 'OpenIMU'
    convert_rads = math.pi /180

    openimu_wrp = OpenIMUros()
    rospy.loginfo("OpenIMU driver initialized.")

    # Persistent time stamp server proxy
    rospy.wait_for_service('set_time_stamp')
    set_time_stamp = rospy.ServiceProxy('set_time_stamp', SetTimeStamp, persistent=True)

    prev = rospy.Time(0,0)
    while not rospy.is_shutdown():
        #read the data - call the get imu measurement data
        readback = openimu_wrp.readimu()
        if readback:

            # Get time stamp from the evaluation board
            us = readback[0]
            ts = rospy.Time(0, us*1000)

            # Check if time stamp should be shared with cameras
            cam_stamp = readback[7]
            if cam_stamp:
                #print("Cam stamp! Storing to server:", str(ts.secs+ts.nsecs/10**9))
                # Request to store on server
                try:
                    resp = set_time_stamp(ts.secs, ts.nsecs)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

        
            diff = ts - prev
            prev = ts
            if diff.to_nsec() > 0:
                print("DIFF!", diff.to_nsec())
                pass

            # Create IMU topic message
            imu_msg.header.stamp = ts
            imu_msg.header.frame_id = frame_id
            imu_msg.header.seq = seq

            imu_msg.orientation_covariance[0] = -1
            imu_msg.linear_acceleration.x = readback[1]
            imu_msg.linear_acceleration.y = readback[2]
            imu_msg.linear_acceleration.z = readback[3]
            imu_msg.linear_acceleration_covariance[0] = -1
            imu_msg.angular_velocity.x = readback[4] * convert_rads
            imu_msg.angular_velocity.y = readback[5] * convert_rads
            imu_msg.angular_velocity.z = readback[6] * convert_rads
            imu_msg.angular_velocity_covariance[0] = -1

            pub_imu.publish(imu_msg)

        seq = seq + 1
        #clk.sleep()
    openimu_wrp.close()         # exit



