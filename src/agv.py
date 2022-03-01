#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from nist_gear.srv import AGVToAssemblyStation
from ariac_utilities import Ariac, Orders
from std_msgs.msg import String

class AGV:
    def __init__(self):
        self.agvs = 4
        self.state_change = ["" for i in range(self.agvs)]
        self.station_change = ["" for i in range(self.agvs)]

    def state_callback(self, data, index):
        if(self.state_change[index-1] != data.data):
            rospy.loginfo("[State: AGV"+str(index)+"]: "+data.data)
            self.state_change[index-1] = data.data

    def station_callback(self, data, index):
        if(self.station_change[index-1] != data.data):
            rospy.loginfo("[Station: AGV"+str(index)+"]: "+data.data)
            self.station_change[index-1] = data.data
    
    def create_state_subscriber(self):
        for index in range(1, self.agvs+1): 
            rospy.Subscriber("/ariac/agv"+str(index)+"/state", String, self.state_callback, index)

    def create_station_subscriber(self):
        for index in range(1, self.agvs+1): 
            rospy.Subscriber("/ariac/agv"+str(index)+"/station", String, self.station_callback, index)
    

def submit_shipment(name, agv, station_id):
    rospy.wait_for_service('/ariac/' + agv + '/submit_shipment')
    shipment_service = rospy.ServiceProxy('/ariac/' + agv + '/submit_shipment', AGVToAssemblyStation)
    shipment_service(station_id, name)

if __name__ == '__main__':
    try:
        rospy.init_node('agv', anonymous=True)
        ariac = Ariac()
        rospy.sleep(50)
        order = Orders()
        submit_shipment(*order.kitting())
        rospy.sleep(5)
        ariac.end()
        rospy.sleep(5)
        rospy.logwarn("All task completed: ROS Shutting down in 10 seconds")
        rospy.sleep(10)
        rospy.signal_shutdown("Task Completed")
        
    except rospy.ROSInterruptException:
        pass
        