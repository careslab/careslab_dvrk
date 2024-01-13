#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class MainSafety:
    def __init__(self):
        rospy.init_node('main_safety', anonymous=True)
        self.subscribers = []
        self.setup_subscribers()

    def setup_subscribers(self):
        # Subscribe to the topics published by EyeBlink, MTMSafety, and PSMSafety
        self.subscribers.append(rospy.Subscriber('eye_blink_topic', String, self.callback))
        self.subscribers.append(rospy.Subscriber('mtm_safety_topic', String, self.callback))
        self.subscribers.append(rospy.Subscriber('psm_safety_topic', String, self.callback))

    def callback(self, data):
        # Callback function for processing incoming messages
        rospy.loginfo(rospy.get_caller_id() + " I heard: %s", data.data)

    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    main_safety_node = MainSafety()
    main_safety_node.spin()