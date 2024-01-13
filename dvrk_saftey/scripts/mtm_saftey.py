#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import threading

class MTMSafety:
    def __init__(self):
        self.pub = rospy.Publisher('mtm_safety_topic', String, queue_size=10)
        rospy.init_node('mtm_safety_node', anonymous=True)
        self.keep_running = True

    def monitor(self):
        while not rospy.is_shutdown() and self.keep_running:
            safety_str = "MTM safety check at %s" % rospy.get_time()
            rospy.loginfo(safety_str)
            self.pub.publish(safety_str)
            rospy.sleep(1.0)

    def start_monitoring(self):
        self.thread = threading.Thread(target=self.monitor)
        self.thread.start()

    def stop_monitoring(self):
        self.keep_running = False
        self.thread.join()

if __name__ == '__main__':
    try:
        mtm_safety_node = MTMSafety()
        mtm_safety_node.start_monitoring()
        rospy.spin()
    except rospy.ROSInterruptException:
        mtm_safety_node.stop_monitoring()
