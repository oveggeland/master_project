#!/usr/bin/env python3

import rospy
from synchronizer.srv import *
import sys
import math
from sensor_msgs.msg import Imu

try:
    from ros_openimu.src.aceinna.tools import OpenIMU
except:  # pylint: disable=bare-except
    temp = (sys.path[0])
    temp2 = temp[0:(len(temp)-7)]
    sys.path.append(temp2 + 'src')
    from aceinna.tools import OpenIMU


class OpenIMUros:
    def __init__(self, options):
        self.openimudev = OpenIMU(**options)
        self.openimudev.startup()

    def close(self):
        self.openimudev.close()

    def readimu(self, packet_type):
        readback = self.openimudev.getdata(packet_type)
        return readback



if __name__ == "__main__":
    rospy.init_node("openimu_driver")

    # Initialize topic publisher
    pub_imu = rospy.Publisher('imu_acc_ar', Imu, queue_size=5)
    imu_msg = Imu()
        
    seq = 0
    frame_id = 'OpenIMU'
    convert_rads = math.pi /180

    # Initialize IMU driver
    options = {'baudrate':230400, 'filter_device_type':'IMU', 'com_port':'auto'}
    openimu_wrp = OpenIMUros(options)
    rospy.loginfo("OpenIMU driver initialized.")

    # Wait for cameras to be ready!
    rospy.wait_for_service('ready_request')    
    cams_ready = rospy.ServiceProxy('ready_request', ReadyRequest)
    while not cams_ready().ready:
        continue

    # Inform IMU that cameras are ready
    print("Cameras are ready! Inform IMU to reset counters and start triggering camera!")
    while openimu_wrp.openimudev.reset_signal(1)['packetType'] != 'success':
        continue

    # Persistent time stamp server proxy
    rospy.wait_for_service('set_time_stamp')    
    set_time_stamp = rospy.ServiceProxy('set_time_stamp', SetTimeStamp, persistent=True)


    while not rospy.is_shutdown():
        #read the data - call the get imu measurement data
        readback = openimu_wrp.readimu('o1')
       
        if readback:
            # Check if time stamp should be shared with cameras
            trigger_flag = readback[9]
            if trigger_flag:
                cam_count = readback[8]
                cam_stamp = rospy.Time(0, 1000*readback[1])
                # Request to store on server
                try:
                    resp = set_time_stamp(cam_stamp.secs, cam_stamp.nsecs, cam_count)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

            # Create IMU topic message
            ts = rospy.Time(0, readback[0]*1000)
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

            seq += 1

    openimu_wrp.close()         # exit



