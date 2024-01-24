# write a ros node that looks up the transform from map to base_link and compares that to a certain point


# x, y, yaw
starting = [1.2953656016185344,5.552489756398731,3.14159]

import rospy
import tf2_ros
import tf

ANGLE_TOLERANCE = 0.05 # radians (about 3 degrees)
DISTANCE_TOLERANCE = 0.05 # meters
MAP_FRAME = "odom"

def main():
    rospy.init_node("check_auto_setup")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        try:
            x_off = False
            y_off = False
            yaw_off = False
            transform = tfBuffer.lookup_transform("odom", "base_link", rospy.Time())
            #print(transform)
            #print(transform.transform.translation.x)
            #print(transform.transform.translation.y)
            # convert quaternion to yaw
            quat = transform.transform.rotation
            yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
            #print(yaw)
            # compare to starting
            if abs(transform.transform.translation.x - starting[0]) > DISTANCE_TOLERANCE:
                x_off = True
                rospy.loginfo_throttle(1, "x off")
            
            if abs(transform.transform.translation.y - starting[1]) > DISTANCE_TOLERANCE:
                y_off = True
                rospy.loginfo_throttle(1, "y off")

            if abs(yaw - starting[2]) > ANGLE_TOLERANCE:
                yaw_off = True
                rospy.loginfo_throttle(1, "yaw off")

            if not x_off and not y_off and not yaw_off:
                rospy.loginfo_throttle(1, "all good")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("no transform found for auto")
            continue


        rate.sleep()


if __name__ == "__main__":
    main()