#! /usr/bin/env python3
import numpy as np
import rospy
from norfair import Detection, Tracker
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point
import tf2_ros
import time

class NorfairNode:
    def publisher(self, tracked_objects: list):
        """
        Tracked objects to ROS message.

        Parameters
        ----------
        tracked_objects : list
            List of tracked objects.
        """
        detection_msg = DetectionsMsg()
        detection_msg.detections = []

        for tracked_object in tracked_objects:
            detection_msg.detections.append(
                DetectionMsg(
                    id=tracked_object.id,
                    label=tracked_object.last_detection.label,
                    scores=[score for score in tracked_object.last_detection.scores],
                    points=[Point(point=point) for point in tracked_object.last_detection.points],
                )
            )
        detection_msg.header.stamp = rospy.get_rostime()
        detection_msg.header.frame_id = "odom"
        
        self.pub.publish(detection_msg)

    def pipeline(self, bbox: DetectionsMsg):
        """
        Generate Norfair detections and pass them to the tracker.

        Parameters
        ----------
        bbox : DetectionsMsg
            DetectionsMsg message from converter.
        """
        # print("Pipeline call back")
        detections = []
        for detection in bbox.detections:
            # print("Detection: ", detection)
            detections.append(
                Detection(
                    points=np.array([detection.points[0].point[0], detection.points[0].point[1]]),
                    scores=np.array(detection.scores),
                    label=detection.label,
                )
            )
        
        
        tracked_objects = self.tracker.update(detections)
        self.tracked_objects  = tracked_objects
        self.publisher(tracked_objects)

    def tf_pub(self, event):
        # publish transform between odom to the tracked notes
        print(f"current time: {rospy.Time.now()}")
        for tracked_object in self.tracked_objects:
            t = tracked_object.last_detection.points[0]
            # print("t: ", t)
            transform = tf2_ros.TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"
            transform.child_frame_id = "track_note_" + str(tracked_object.id)
            transform.transform.translation.x = t[0]
            transform.transform.translation.y = t[1]
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            self.br.sendTransform(transform)
            print(f"publishing transform: {transform}")

    def main(self):
        """
        Norfair initialization and subscriber and publisher definition.
        """
        rospy.init_node("norfair_ros")

        # Load parameters
        publishers = rospy.get_param("norfair_publishers")
        subscribers = rospy.get_param("norfair_subscribers")
        norfair_setup = rospy.get_param("norfair_setup")
        converter = subscribers["converter"]
        norfair_detections = publishers["detections"]

        # Norfair tracker initialization
        self.tracker = Tracker(
            distance_function=norfair_setup["distance_function"],
            distance_threshold=norfair_setup["distance_threshold"],
            hit_counter_max=norfair_setup["hit_counter_max"],
            initialization_delay=norfair_setup["initialization_delay"],
            pointwise_hit_counter_max=norfair_setup["pointwise_hit_counter_max"],
            detection_threshold=norfair_setup["detection_threshold"],
            past_detections_length=norfair_setup["past_detections_length"],
        )
        self.tracked_objects = []

        # ROS subscriber and publisher definition
        self.pub = rospy.Publisher(
            norfair_detections["topic"], DetectionsMsg, queue_size=norfair_detections["queue_size"]
        )
        rospy.Subscriber(converter["topic"], DetectionsMsg, self.pipeline)
        # make a tf publisher
        
        self.br = tf2_ros.TransformBroadcaster()
        # make a timer to publish the transform every 0.1 seconds
        rospy.Timer(rospy.Duration(0.1), self.tf_pub, oneshot=False)

        rospy.spin()


if __name__ == "__main__":
    try:
        NorfairNode().main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Norfair node terminated.")
