#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from nist_gear.srv import AGVToAssemblyStation
from ariac_utilities import Ariac, Orders
from std_msgs.msg import String


class AGV:
    """
      A class to represent AGV.

      ...

      Methods
      -------
      state_callback(self, data, index):
      Callback for the state of the AGV

      station_callback(self, data, index):
      Callback for the assembly station location

      create_state_subscriber(self):
      Creates subscriber for Agv state

      create_station_subscriber(self):
      Creates subscriber for Agv station

      submit_shipment(name, agv, station_id):
      Submits Agv shipment to station

      """
    def __init__(self):
        """
        init method when invoked assigns initial values to the parameters
        agvs, state_change and station_change

        Parameters:
        agvs: Stores number of AGVs in the ARIAC contest
        state_change: initially stores empty string
        station_change: initially stores empty string

        """
        self.agvs = 4
        self.state_change = ["" for i in range(self.agvs)]
        self.station_change = ["" for i in range(self.agvs)]

    def state_callback(self, data, index):
        """
        The state of the AGV is printed on the screen and stored.

        Returns
        -------
        None
        """
        if self.state_change[index-1] != data.data:
            rospy.loginfo("[State: AGV"+str(index)+"]: "+data.data)
            self.state_change[index-1] = data.data

    def station_callback(self, data, index):
        """
        The assembly station of the AGV is printed on the screen and stored.

        Returns
        -------
        None
        """
        if self.station_change[index-1] != data.data:
            rospy.loginfo("[Station: AGV"+str(index)+"]: "+data.data)
            self.station_change[index-1] = data.data
    
    def create_state_subscriber(self):
        """
        Creates the state subscriber for the AGV.

        Returns
        -------
        None
        """
        for index in range(1, self.agvs+1):
            rospy.Subscriber("/ariac/agv"+str(index)+"/state", String, self.state_callback, index)

    def create_station_subscriber(self):
        """
        Creates the station subscriber for the AGV.

        Returns
        -------
        None
        """
        for index in range(1, self.agvs+1):
            rospy.Subscriber("/ariac/agv"+str(index)+"/station", String, self.station_callback, index)
    

def submit_shipment(name, agv, station_id):
    """
    This method assigns Agv shipment to the station.

    Returns
    -------
    None
    """
    rospy.wait_for_service('/ariac/' + agv + '/submit_shipment')
    shipment_service = rospy.ServiceProxy('/ariac/' + agv + '/submit_shipment', AGVToAssemblyStation)
    shipment_service(station_id, name)


if __name__ == '__main__':
    try:

        rospy.init_node('agv', anonymous=True)
        ariac = Ariac()
        if(ariac.status()):
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
        