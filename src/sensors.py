#!/usr/bin/env python

import rospy
from nist_gear.msg import Proximity, LogicalCameraImage
from sensor_msgs.msg import PointCloud , LaserScan, Range
from ariac_utilities import Ariac
from agv import AGV


class Sensors:
    """
      A class to represent sensors.

       ...

       Methods
       -------
       logical_camera_bins0_callback(data):
       Callback for logical camera bins0 sensor

       logical_camera_station2_callback(data):
       Callback for logical camera station2 sensor

       depth_camera_bins1_callback(data):
       Callback for depth camera bins1

       laser_profiler_0_callback(data):
       Callback for laser profiler 0

       proximity_sensor_0_callback(data):
       Callback for proximity sensor 0

       breakbeam_0_callback(data):
       callback for breakbeam sensor

       breakbeam_0_change_callback(data):
       Callback for breakbeam 0 change sensor

       quality_control_sensor_1_callback(data):
       Callback for quality control sensor 1

       quality_control_sensor_2_callback(data)
       Callback for quality control sensor 2

       quality_control_sensor_3_callback(data)
       Callback for quality control sensor 3

       quality_control_sensor_4_callback(data)
       Callback for quality control sensor 4

    """
    def logical_camera_bins0_callback(self, data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        rospy.loginfo_throttle(5, "Callback triggered for Topic /ariac/logical_camera_bins0")

    def logical_camera_station2_callback(self, data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        rospy.loginfo_throttle(5, "Callback triggered for Topic /ariac/logical_camera_station2")

    def depth_camera_bins1_callback(self, data):
        """
        Depth camera provides a point cloud of sensed distances.

        Returns
        -------
        None
        """
        pass

    def laser_profiler_0_callback(self, data):
        """
        Laser scanner provides an array of distances to a sensed object.

        Returns
        -------
        None
        """
        rospy.loginfo_throttle(5, "Callback triggered for Topic /ariac/laser_profiler_0")

    def proximity_sensor_0_callback(self, data):
        """
        Proximity sensor reports how far an object is from the sensor.

        Returns
        -------
        None
        """
        if(data.range < 0.12):
            rospy.loginfo("Callback triggered for Topic /ariac/proximity_sensor_0")

    def breakbeam_0_callback(self, data):
        """
        This callback method prints a statement on the terminal when a beam
        is broken by an object .

        Returns
        -------
        None
        """
        if(data.object_detected):
            rospy.loginfo("Callback triggered for Topic /ariac/breakbeam_0")

    def breakbeam_0_change_callback(self, data):
        """
        This callback method prints a statement on the terminal when a beam
        is broken by an object .

        Returns
        -------
        None
        """
        if(data.object_detected):
            rospy.loginfo("Callback triggered for Topic /ariac/breakbeam_0_change")

    def quality_control_sensor_1_callback(self, data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        rospy.loginfo_throttle(5,"Callback triggered for Topic /ariac/quality_control_sensor_1")

    def quality_control_sensor_2_callback(self, data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        rospy.loginfo_throttle(5,"Callback triggered for Topic /ariac/quality_control_sensor_2")

    def quality_control_sensor_3_callback(self, data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        rospy.loginfo_throttle(5,"Callback triggered for Topic /ariac/quality_control_sensor_3")

    def quality_control_sensor_4_callback(self, data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        rospy.loginfo_throttle(5,"Callback triggered for Topic /ariac/quality_control_sensor_4")


    def create_sensor_subscribers(self):
        """
            In this method the sensor subscribers are created.

            Returns
            -------
            None
        """
        rospy.Subscriber("/ariac/logical_camera_bins0", LogicalCameraImage, self.logical_camera_station2_callback)
        rospy.Subscriber("/ariac/logical_camera_station2", LogicalCameraImage, self.depth_camera_bins1_callback)
        rospy.Subscriber("/ariac/depth_camera_bins1", PointCloud, self.depth_camera_bins1_callback)
        rospy.Subscriber("/ariac/laser_profiler_0", LaserScan, self.laser_profiler_0_callback)
        rospy.Subscriber("/ariac/proximity_sensor_0", Range, self.proximity_sensor_0_callback)
        rospy.Subscriber("/ariac/breakbeam_0", Proximity, self.breakbeam_0_callback)
        rospy.Subscriber("/ariac/breakbeam_0_change", Proximity, self.breakbeam_0_change_callback)
        rospy.Subscriber("/ariac/quality_control_sensor_1", LogicalCameraImage, self.quality_control_sensor_1_callback)
        rospy.Subscriber("/ariac/quality_control_sensor_2", LogicalCameraImage, self.quality_control_sensor_2_callback)
        rospy.Subscriber("/ariac/quality_control_sensor_3", LogicalCameraImage, self.quality_control_sensor_3_callback)
        rospy.Subscriber("/ariac/quality_control_sensor_4", LogicalCameraImage, self.quality_control_sensor_4_callback)

        
if __name__ == '__main__':
    try:
        rospy.init_node('sensors', anonymous=True)
        ariac = Ariac()
        sensors = Sensors()
        agv =AGV()
        # Start competition
        if(ariac.status):
            sensors.create_sensor_subscribers()
            agv.create_state_subscriber()
            agv.create_station_subscriber()
            

            # rospy.Subscriber("/ariac/logical_camera_bins0", LogicalCameraImage , callback)
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
        
        
        
        
        
        


