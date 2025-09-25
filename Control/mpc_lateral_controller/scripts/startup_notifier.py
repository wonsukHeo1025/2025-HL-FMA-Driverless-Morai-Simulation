#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Log

def main():
    rospy.init_node('startup_notifier', anonymous=True)
    
    message = rospy.get_param('~message', 'System started')
    once = rospy.get_param('~once', True)
    
    pub = rospy.Publisher('/rosout', Log, queue_size=1, latch=True)
    
    rate = rospy.Rate(1)
    
    rospy.sleep(0.5)
    
    log_msg = Log()
    log_msg.header.stamp = rospy.Time.now()
    log_msg.level = Log.INFO
    log_msg.name = rospy.get_name()
    log_msg.msg = message
    log_msg.file = __file__
    log_msg.function = 'main'
    log_msg.line = 0
    
    pub.publish(log_msg)
    rospy.loginfo(message)
    
    if once:
        rospy.sleep(0.5)
        rospy.signal_shutdown('Message published')
    else:
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass