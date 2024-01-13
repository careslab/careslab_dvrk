#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import threading

class EyeBlink:
    def __init__(self):
        self.pub = rospy.Publisher('eye_blink_topic', String, queue_size=10)
        rospy.init_node('eye_blink_node', anonymous=True)
        self.keep_running = True

    def spin(self):        
        blink_str = "Eye blink at %s" % rospy.get_time()
        # rospy.loginfo(blink_str)
        self.pub.publish(blink_str)            
        rospy.spin()
        

if __name__ == '__main__':
    try:
        eye_blink_node = EyeBlink()
        eye_blink_node.spin()
        rospy.spin()
    except rospy.ROSInterruptException:        
