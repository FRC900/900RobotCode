#!/usr/bin/env python3

import rospy
from apriltag_msgs.msg import ApriltagArrayStamped

NUMBER_OF_TAGS = 16 # Field has tags 1 - 16, TODO make this configurable
tags_seen = []
tag_groups_seen = []

def callback(tag_msg: ApriltagArrayStamped):
    #  in the callback, for each tag id seen,
    tag_ids_seen = [detection.id for detection in tag_msg.apriltags]

    for tag_id in tag_ids_seen:
        # set the list entry for that tag to true
        tags_seen[tag_id] = True

    #  print out the list of tags not seen
    print("Tags not seen: ", end="")
    for i in range(1, NUMBER_OF_TAGS + 1):
        if tags_seen[i] == False:
            print(f"{i} ", end="")
    print()

    # Add new tags to a new group
    for tag_id in tag_ids_seen:
        tag_seen_in_group = False
        for group in tag_groups_seen:
            if tag_id in group:
                tag_seen_in_group = True
                break
        if not tag_seen_in_group:
            tag_groups_seen.append(set([tag_id]))

    # Create valid_groups[], a list of bools
    # It should be initialized to a list of True values, one per group
    # If we merge a group later, set one of the list entries for the pair to false
    # After merging, we can update the list of groups to only ones that remain true


    # Go through each pair ot tags seen
    # If the tags are in different groups, merge the groups

def main():
    rospy.init_node('check_tags', anonymous=True)
    rospy.Subscriber('/tag_detections', ApriltagArrayStamped, callback)

    # keep track of tags seen
    #  store a list of bool values, one per tag
    #     initialize the list to false for each tag
    for _ in range(NUMBER_OF_TAGS + 1):
        tags_seen.append(False)

    rospy.spin()
    
if __name__ == '__main__':
    main()
    