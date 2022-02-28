#!/usr/bin/env python

import rospy
from nist_gear.msg import Proximity, LogicalCameraImage
from sensor_msgs.msg import PointCloud , LaserScan, Range
from ariac_utilities import Ariac

class Sensors:
    def logical_camera_bins0_callback(self, data):
        rospy.loginfo_throttle(5, "Callback triggered for Topic /ariac/logical_camera_bins0")

    def logical_camera_station2_callback(self, data):
        rospy.loginfo_throttle(5, "Callback triggered for Topic /ariac/logical_camera_station2")

    def depth_camera_bins1_callback(self, data):
        pass

    def laser_profiler_0_callback(self, data):
        rospy.loginfo_throttle(5, "Callback triggered for Topic /ariac/laser_profiler_0")

    def proximity_sensor_0_callback(self, data):
        if(data.range < 0.12):
            rospy.loginfo("Callback triggered for Topic /ariac/proximity_sensor_0")

    def breakbeam_0_callback(self, data):
        if(data.object_detected):
            rospy.loginfo("Callback triggered for Topic /ariac/breakbeam_0")

    def breakbeam_0_change_callback(self, data):
        if(data.object_detected):
            rospy.loginfo("Callback triggered for Topic /ariac/breakbeam_0_change")

    def quality_control_sensor_1_callback(self, data):
        rospy.loginfo_throttle(5,"Callback triggered for Topic /ariac/quality_control_sensor_1")

    def quality_control_sensor_2_callback(self, data):
        rospy.loginfo_throttle(5,"Callback triggered for Topic /ariac/quality_control_sensor_2")

    def quality_control_sensor_3_callback(self, data):
        rospy.loginfo_throttle(5,"Callback triggered for Topic /ariac/quality_control_sensor_3")

    def quality_control_sensor_4_callback(self, data):
        rospy.loginfo_throttle(5,"Callback triggered for Topic /ariac/quality_control_sensor_4")


    def create_sensor_subscribers(self):
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

        # Start competition
        ariac.start()
        sensors.create_sensor_subscribers()
        

        # rospy.Subscriber("/ariac/logical_camera_bins0", LogicalCameraImage , callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        
        
        
        
        
        


