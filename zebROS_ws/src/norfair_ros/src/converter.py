#! /usr/bin/env python3
import rospy
from norfair_ros.msg import BoundingBoxes
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point
from field_obj.msg import Detection as FieldDet
from field_obj.msg import TFDetection
import tf2_ros
import tf2_geometry_msgs
import message_filters

class Converter:
    """
    The Converter class is a ROS node that converts different input messages to a norfair_ros input message.
    """

    def boundingboxes_to_norfair(self, bboxes: BoundingBoxes) -> None:
        """
        Convert BoundingBoxes message to DetectionsMsg message.

        Parameters
        ----------
        bboxes : BoundingBoxes
            BoundingBoxes message from darknet_ros.
        """
        detections = []
        for bbox in bboxes.bounding_boxes:
            detections.append(
                DetectionMsg(
                    id=0,
                    label=bbox.Class,
                    #scores=[bbox.probability, bbox.probability],
                    scores=[1.0, 1.0],
                    points=[
                        Point([bbox.xmin, bbox.ymin]),
                        Point([bbox.xmax, bbox.ymax]),
                    ],
                )
            )

        detections_msg = DetectionsMsg()
        detections_msg.detections = detections

        self.converter_publisher.publish(detections_msg)

    def field_det_to_norfair(self, bboxes: TFDetection) -> None:
        # print("callback")
        detections = []
        for bbox in bboxes.objects:
            detections.append(
                DetectionMsg(
                    id=0,
                    label=bbox.label,
                    scores=[bbox.confidence, bbox.confidence],
                    points=[
                        Point([bbox.tl.x, bbox.tl.y]),
                        Point([bbox.br.x, bbox.br.y]),
                    ],
                )
            )

        detections_msg = DetectionsMsg()
        detections_msg.detections = detections
        detections_msg.header.stamp = bboxes.header.stamp
        # print(f"Publishing {detections_msg}")
        self.converter_publisher.publish(detections_msg)

    def screen_to_world(self, field_dets1: FieldDet, field_dets2: FieldDet, field_dets3: FieldDet) -> None:
        #print("screen to world callback")

        # print(field_dets)
        detections_msg = DetectionsMsg()
        detections_msg.detections = []
        detections_msg.header.stamp = field_dets1.header.stamp
        detections_msg.header.frame_id = "odom"

        for field_dets in [field_dets1, field_dets2, field_dets3]:
            try:
                transform = self.tfBuffer.lookup_transform("odom", field_dets.header.frame_id, field_dets.header.stamp, rospy.Duration(0.02)) # this shouldn't break anything IRL but needed for sim
            except:
                rospy.logerr("failed transform in converter")
                return

            for detection in field_dets.objects:
                #rospy.loginfo(f"Detection {detection}")
                # transform the point from zed_objdetect_left_camera_frame to map
                p = tf2_geometry_msgs.PointStamped()
                p.point.x = detection.location.x
                p.point.y = detection.location.y
                
                res = tf2_geometry_msgs.do_transform_point(p, transform)
                if detection.id.isdigit():
                    detection.id = "tag_" + detection.id

                detections_msg.detections.append(
                    DetectionMsg(
                        id=0,
                        label=detection.id,
                        scores=[1.0],
                        points=[
                            Point([res.point.x, res.point.y]),
                        ],
                    )
                )
            
        #print(f"\n\n=================Publishing {detections_msg}")
        
        self.converter_publisher.publish(detections_msg)
 
    def main(self) -> None:
        rospy.init_node("converter")
        # Load parameters
        subscribers = rospy.get_param("converter_subscribers")
        publishers = rospy.get_param("converter_publishers")
        screen_to_world_det = subscribers["screen_to_world"]
        tag_to_world_back = subscribers["tag_to_world_back"]
        tag_to_world_front = subscribers["tag_to_world_front"]
        

        output = publishers["output"]

        # ROS subscriber definition
        # rospy.Subscriber(screen_to_world_det["topic"], FieldDet, self.screen_to_world)
        # rospy.Subscriber(tag_to_world_back["topic"], FieldDet, self.screen_to_world)
        # rospy.Subscriber(sim_tags["topic"], FieldDet, self.screen_to_world)

        front_tag_sub = message_filters.Subscriber(tag_to_world_front["topic"], FieldDet)
        back_tag_sub = message_filters.Subscriber(tag_to_world_back["topic"], FieldDet)
        screen_to_world_sub = message_filters.Subscriber(screen_to_world_det["topic"], FieldDet)

        ts = message_filters.ApproximateTimeSynchronizer([front_tag_sub, back_tag_sub, screen_to_world_sub], 10, 0.1)
        ts.registerCallback(self.screen_to_world)



        self.converter_publisher = rospy.Publisher(
            output["topic"], DetectionsMsg, queue_size=output["queue_size"]
        )
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.spin()


if __name__ == "__main__":
    try:
        print("running")
        Converter().main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Converter node terminated.")
