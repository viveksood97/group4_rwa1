#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from nist_gear.srv import AGVToAssemblyStation
from ariac_utilities import Ariac, Orders


def submit_shipment(name, agv, station_id):
    print(name, agv, station_id)
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
        