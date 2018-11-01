#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

def callback(data):
    publisher.publish(String('I heard: {}'.format(data.data)))

def listener():
    global publisher
    # init node
    rospy.init_node('assignment1_publisher_subscriber', anonymous=True)
    # create publisher
    publisher = rospy.Publisher('/assignment1_publisher_subscriber', String, queue_size = 10)
    # subscribe with callback
    rospy.Subscriber('/yaw', Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
