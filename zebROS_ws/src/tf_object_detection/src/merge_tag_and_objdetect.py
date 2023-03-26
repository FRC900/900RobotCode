#! /usr/bin/env python3

import rospy
import message_filters
from field_obj.msg import Detection

def callback(obj_msg, tag_msg):
  out_msg = Detection()
  out_msg.header.stamp = obj_msg.header.stamp
  out_msg.header.frame_id = obj_msg.header.frame_id

  for o in obj_msg.objects:
    if 'april_tag' not in o.id:
      out_msg.objects.append(o)
  for t in tag_msg.objects:
    t.id = "april_tag_" + t.id
    out_msg.objects.append(t)

  global pub
  pub.publish(out_msg)
    
def main():
  rospy.init_node('talker', anonymous=True)
  objdetect_sub = message_filters.Subscriber('/tf_object_detection/object_detection_world', Detection)
  tag_sub = message_filters.Subscriber('/tf_object_detection/tag_detection_world', Detection)

  ts = message_filters.ApproximateTimeSynchronizer([objdetect_sub, tag_sub], queue_size=10, slop=0.1)
  ts.registerCallback(callback)
  global pub
  pub = rospy.Publisher('/tf_object_detection/combined_detection_world', Detection, queue_size=2)
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass