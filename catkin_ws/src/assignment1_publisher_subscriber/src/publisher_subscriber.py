#!/usr/bin/env python
import rospy
from std_msgs.msg import String

pub = None

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Heard %s', data.data)
    pub.publish(std_msgs.msg.String('I heard: {}'.format(data.data)))

def listener():
    # init node
    rospy.init_node('assignment1_publisher_subscriber', anonymous=True)
    # create publisher
    pub = rospy.Publisher('/assignment1_publisher_subscriber', std_msgs.String, queue_size = 10)
    # subscribe with callback
    rospy.Subscriber('/yaw', std_msgs.Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
