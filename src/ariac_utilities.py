#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from nist_gear.msg import Order
from std_msgs.msg import String


class Orders:
    """
    A class to represent orders.
    Order is an instruction containing kits or assembly products for the robot system to complete

    ...

    Methods
    -------
    start():
    starts the ariac contest.
    end():
    ends the ariac contest
    """

    def __init__(self):
        """
        Init function assigns initial values to order and assembly orders
        """
        self.order = rospy.wait_for_message('/ariac/orders', Order)
        self.assembly_orders = self.order.assembly_shipments
    
    def kitting(self):
        """
        Here the kitting action takes place. The parts are collected and placed on Agv tray.

        Returns
        -------
        shipment_type: Stores type of shipment
        agv_id: stores agv id
        station_id: stores station id
        """
        kitting_orders = self.order.kitting_shipments[0] #will we get kitting and assembly both in orders?
        shipment_type = kitting_orders.shipment_type  #what does shipment submission mean?
        agv_id = kitting_orders.agv_id
        station_id = kitting_orders.station_id
        return shipment_type, agv_id, station_id
    def Assembly(self):
        pass


class Ariac:
    """
    A class to start and stop competition.

    Methods
    -------
    start():
    starts the ariac contest.
    end():
    ends the ariac contest
    """
    def start(self):
        """
        The ARIAC contest is started by triggering the service /ariac/start_competition.

        Returns
        -------
        None
        """
        # Wait for service to be active
        rospy.loginfo("Waiting for the competition to be ready...")
        rospy.wait_for_service('/ariac/start_competition')
        start_service = rospy.ServiceProxy('/ariac/start_competition', Trigger)
        
        # Success
        rospy.loginfo("Competition is now ready.")
        rospy.loginfo("Requesting competition start...")
        # Call start
        resp = start_service()
        # Check if successful
        if (resp.success != True):
            rospy.logerr("Failed to start the competition: " + resp.message);
        else:
            rospy.loginfo("Competition started!")
    
    def end(self):
        """
        The ARIAC contest is ended by triggering the service /ariac/stop_competition.

        Returns
        -------
        None
        """
        # Wait for service to be active
        rospy.wait_for_service('/ariac/end_competition')
        end_service = rospy.ServiceProxy('/ariac/end_competition', Trigger)
        
        # Success
        rospy.loginfo("Competition is ending now")
        rospy.loginfo("Requesting competition end...")
        # Call start
        resp = end_service()
        # Check if successful
        if not resp.success:
            rospy.logerr("Failed to end the competition: " + resp.message);
        else:
            rospy.loginfo("Competition ended!")
    
    def status(self):
        competition_status = rospy.wait_for_message("/ariac/competition_state", String, timeout=None).data
        print(competition_status)
        if(competition_status == "init"):
            self.start()
            return True
        elif(competition_status == "go"):
            return True
        elif(competition_status == "done"):
            rospy.loginfo("Competition has ended")
            return False

        
        
        

    
    