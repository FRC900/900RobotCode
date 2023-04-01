#!/usr/bin/env python3
import cv2
import numpy as np
from frc_msgs.msg import ButtonBoxState2023
import rospy
blank_image = np.zeros((1,1,3), np.uint8)

prev_msg = ButtonBoxState2023()

def bindKey(received_key, key, field, msg):
    exec(f"""
if received_key == ord('{key}'):
    if prev_msg.{field}Press or prev_msg.{field}Button:
        msg.{field}Button = True
    else:
        msg.{field}Press = True
        print(f"{field}")
else:
    if prev_msg.{field}Button:
        msg.{field}Release = True""")
    
    return msg

rospy.init_node("pubthingy", anonymous=True)

pub = rospy.Publisher("/frcrobot_rio/button_box_states", ButtonBoxState2023, queue_size=1)

last_up_down_mid = "Up"
up_down_mid = "Up"

last_left_right_mid = "Left"
left_right_mid = "Left"

while True and not rospy.is_shutdown():
    cv2.imshow("button box sim", blank_image)
    key = cv2.waitKey(1000)

    msg = ButtonBoxState2023()

    msg = bindKey(key, 'f', "gridSelectConeLeft", msg)
    print(up_down_mid, left_right_mid)
    msg = bindKey(key, 'g', "gridSelectCube", msg)
    msg = bindKey(key, 'h', "gridSelectConeRight", msg)

    msg.lockingSwitchButton = True
    
    if key == ord('q'):
        up_down_mid = "Up"
    if key == ord('a'):
        up_down_mid = ""
    if key == ord('z'):
        up_down_mid = "Down"

    if last_up_down_mid != up_down_mid:
        try:
            exec(f"msg.heightSelectSwitch{last_up_down_mid}Release = True")
        except: pass
        try:
            exec(f"msg.heightSelectSwitch{up_down_mid}Press = True")
        except: pass

    else:
        try:
            exec(f"msg.heightSelectSwitch{up_down_mid}Button = True")
        except: pass

    if key == ord('i'):
        left_right_mid = "Left"
    if key == ord('o'):
        left_right_mid = ""
    if key == ord('p'):
        left_right_mid = "Right"

    if last_left_right_mid != left_right_mid:
        try:
            exec(f"msg.heightSelectSwitch{last_left_right_mid}Release = True")
        except: pass
        try:
            exec(f"msg.heightSelectSwitch{left_right_mid}Press = True")
        except: pass

    else:
        try:
            exec(f"msg.heightSelectSwitch{left_right_mid}Button = True")
        except: pass
    
    last_left_right_mid = left_right_mid
    last_up_down_mid = up_down_mid

    msg = bindKey(key, 'x', "bottomRightWhite", msg)

    msg = bindKey(key, ' ', "red", msg)

    if (key == ord('`')):
        break

    prev_msg = msg

    pub.publish(msg)

