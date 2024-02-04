#! /usr/bin/env python3
import rospy



from frc_msgs.msg import RumbleState
from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback

def main():
    rospy.init_node("sim_rumble", anonymous =True)
    # rospy.subscriber("rumble_states", RumbleState, callback)
    rospy.Subscriber("/frcrobot_rio/rumble_states", RumbleState, callback)
    global pub 
    pub = rospy.Publisher("set_feedback", JoyFeedbackArray, queue_size=1)
    rospy.spin ()

def callback(msg):
    out_msg = JoyFeedbackArray()
    out_msg.array.append(JoyFeedback())
    out_msg.array[0].type = JoyFeedback.TYPE_RUMBLE
    out_msg.array[0].id = 0
    out_msg.array[0].intensity = msg.left[0]/ float((1 << 16) -1)
    out_msg.array.append(JoyFeedback())
    out_msg.array[1].type = JoyFeedback.TYPE_BUZZER
    out_msg.array[1].id = 0
    out_msg.array[1].intensity = msg.right[0]/ float((1 << 16) -1)
    global pub 
    pub.publish(out_msg)

if __name__ == '__main__':
    main()