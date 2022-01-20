from __future__ import print_function
from synchronizer.srv import SetTimeStamp, SetTimeStampResponse, GetTimeStamp, GetTimeStampResponse
import rospy

class TimeStamper():
    def __init__(self) -> None:
        self.ts = rospy.Time(0)
        self.clients = {}
    
    def get_time(self, id):
        if id not in self.clients.keys():
            self.clients[id] = 0
        
        if self.clients[id] == 0:
            self.clients[id] = 1
            return self.ts.secs, self.ts.nsecs
        else:
            return 0, 0

    def set_time(self, secs, nsecs):
        self.ts = rospy.Time(secs, nsecs)
        for client in self.clients.keys():
            self.clients[client] = 0


def handle_set_time_stamp(req, time_stamper):
    time_stamper.set_time(req.secs, req.nsecs)
    return True

def handle_get_time_stamp(req, time_stamper):
    return time_stamper.get_time(req._connection_header['id'])

def time_stamp_server():
    rospy.init_node('time_stamp_server')
    time_stamper = TimeStamper()
    s = rospy.Service('set_time_stamp', SetTimeStamp, lambda msg: handle_set_time_stamp(msg, time_stamper))
    g = rospy.Service('get_time_stamp', GetTimeStamp, lambda msg: handle_get_time_stamp(msg, time_stamper))

    print("Server ready")
    rospy.spin()

if __name__ == "__main__":
    time_stamp_server()