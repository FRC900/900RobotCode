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
        # print(f"Publishing {detections_msg}")
        self.converter_publisher.publish(detections_msg)

    def screen_to_world(self, field_dets: FieldDet) -> None:
        # print("screen to world callback")
        detections = []
        # print(field_dets)
        transform = self.tfBuffer.lookup_transform("map", field_dets.header.frame_id, rospy.Time())

        for detection in field_dets.objects:
            # transform the point from zed_objdetect_left_camera_frame to map
            p = tf2_geometry_msgs.PointStamped()
            p.point.x = detection.location.x
            p.point.y = detection.location.y
            
            res = tf2_geometry_msgs.do_transform_point(p, transform)
            if detection.id == "note":
                #print(f"\ninital point {detection.location}\ntransformed point {res.point}")
                pass
            detections.append(
                DetectionMsg(
                    id=0,
                    label=detection.id,
                    scores=[1.0],
                    points=[
                        Point([res.point.x, res.point.y]),
                    ],
                )
            )
        
        detections_msg = DetectionsMsg()
        detections_msg.detections = detections
        #print(f"\n\n=================Publishing {detections_msg}")
        
        self.converter_publisher.publish(detections_msg)
 
    def main(self) -> None:
        rospy.init_node("converter")
        # Load parameters
        subscribers = rospy.get_param("converter_subscribers")
        publishers = rospy.get_param("converter_publishers")
        screen_to_world_det = subscribers["screen_to_world"]
        output = publishers["output"]

        # ROS subscriber definition
        rospy.Subscriber(screen_to_world_det["topic"], FieldDet, self.screen_to_world)
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
