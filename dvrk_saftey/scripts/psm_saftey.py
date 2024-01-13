#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import threading

class PSMSafety:
    def __init__(self):
        self.pub = rospy.Publisher('psm_safety_topic', String, queue_size=10)
        rospy.init_node('psm_safety_node', anonymous=True)
        self.keep_running = True

    def monitor(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown() and self.keep_running:
            safety_str = "PSM safety check at %s" % rospy.get_time()
            rospy.loginfo(safety_str)
            self.pub.publish(safety_str)
            rate.sleep()
    
if __name__ == '__main__':
    try:
        psm_safety_node = PSMSafety()
        psm_safety_node.monitor()        
    except rospy.ROSInterruptException:        

