from __future__ import print_function
from synchronizer.srv import SetTimeStamp, GetTimeStamp, ReadySignal, ReadyRequest
import rospy
import sys
import numpy as np

class TimeStamper():
    def __init__(self, n_cams) -> None:
        self.n_cams = n_cams
        self.stamps = np.empty(65536, rospy.Time)
        self.clients_ready = np.zeros(self.n_cams).astype(int)

    
    def get_time(self, seq):
        ts = self.stamps[seq]
        try:
            return ts.secs, ts.nsecs
        except:
            return 0, 0

    def set_time(self, secs, nsecs, seq):
        ts = rospy.Time(secs, nsecs)
        self.stamps[seq] = ts
        return 1

    def camera_ready(self, id):
        self.clients_ready[id] = 1


def handle_set_time_stamp(req, time_stamper):
    time_stamper.set_time(req.secs, req.nsecs, req.seq)
    return True

def handle_get_time_stamp(req, time_stamper):
    return time_stamper.get_time(req.seq)

def handle_ready_signal(req, time_stamper):
    time_stamper.clients_ready[req.id] = 1
    return True

def handle_ready_request(req, time_stamper):
    return time_stamper.clients_ready.sum() == time_stamper.n_cams


def time_stamp_server(n_cams=2):
    rospy.init_node('time_stamp_server')
    time_stamper = TimeStamper(n_cams)
    s = rospy.Service('set_time_stamp', SetTimeStamp, lambda msg: handle_set_time_stamp(msg, time_stamper))
    g = rospy.Service('get_time_stamp', GetTimeStamp, lambda msg: handle_get_time_stamp(msg, time_stamper))
    rs = rospy.Service('ready_signal', ReadySignal, lambda msg: handle_ready_signal(msg, time_stamper))
    rr = rospy.Service('ready_request', ReadyRequest, lambda msg: handle_ready_request(msg, time_stamper))

    print("Server ready")
    rospy.spin()

if __name__ == "__main__":
    time_stamp_server(int(sys.argv[1]))