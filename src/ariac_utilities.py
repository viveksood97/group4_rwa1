#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from nist_gear.msg import Order

class Orders:                                       #do we only get one order?
    def __init__(self):
        self.order = rospy.wait_for_message('/ariac/orders', Order)
        self.assembly_orders = self.order.assembly_shipments
    
    def kitting(self):
        kitting_orders = self.order.kitting_shipments[0] #will we get kitting and assembly both in orders?
        shipment_type = kitting_orders.shipment_type  #what does shipment submission mean?
        agv_id = kitting_orders.agv_id
        station_id = kitting_orders.station_id
        return shipment_type, agv_id, station_id

class Ariac:
    def start(self):
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
        # Wait for service to be active
        rospy.wait_for_service('/ariac/end_competition')
        end_service = rospy.ServiceProxy('/ariac/end_competition', Trigger)
        
        # Success
        rospy.loginfo("Competition is ending now")
        rospy.loginfo("Requesting competition end...")
        # Call start
        resp = end_service()
        # Check if successful
        if (resp.success != True):
            rospy.logerr("Failed to end the competition: " + resp.message);
        else:
            rospy.loginfo("Competition ended!")
    
    