#!/usr/bin/env python3
import hashlib
import os
import rospy

def file_changed(f):
    sum_path = f.replace(".", "")
    sum_path = ".md5sum" + str(f) + ".txt"
    if not os.path.exists(f):
        rospy.logwarn("pb file " + f + " not found")
        return True
    if not os.path.exists(sum_path):
        rospy.logwarn("sum file not found")
        with open(f, "rb") as file:
            old_sum = hashlib.md5(file.read()).hexdigest()
        with open(sum_path, "w") as file:
            file.write(old_sum)
        return True

    else:
        with open(sum_path, 'r') as sum_file:
            old_sum = sum_file.read()
        rospy.logwarn("sum file found, old sum = " + old_sum)
    
    sum = hashlib.md5()
    with open(f, 'rb') as in_file:
        for chunk in iter(lambda: in_file.read(4096), b''):
            sum.update(chunk)
    text_sum = sum.hexdigest()
    rospy.logwarn("New sum = " + str(text_sum))
    if text_sum == old_sum:
        rospy.logwarn("Match")
        return False
    with open(sum_path, 'w') as sum_out:
        sum_out.write(text_sum)
        rospy.logwarn("Mismatch")
        return True

    
