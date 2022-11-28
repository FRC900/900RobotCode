#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

global pub
def callback(msg): 
    data = msg.data
    data = data + "!!!!!"
    # callback, reads in message from talker
    # modifies it
    # Then publishes in the callback
    pub.publish(data)

def main():
    global pub
    rospy.init_node('middle', anonymous=True)
    pub = rospy.Publisher('new_chatter', String, queue_size=10)
    rospy.Subscriber("chatter", String, callback)
    
    rospy.spin()

if __name__ == "__main__":
    main()