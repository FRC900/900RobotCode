#!/usr/bin/env python3


import rospy
from apriltag_msgs.msg import ApriltagArrayStamped

NUMBER_OF_TAGS = 16 # Field has tags 1 - 16, TODO make this configurable
tags_seen = []
tag_groups_seen = [] #contains

def callback(tag_msg: ApriltagArrayStamped):
    #  in the callback, for each tag id seen,
    tag_ids_seen = [detection.id for detection in tag_msg.apriltags if detection.id in range(1, NUMBER_OF_TAGS+1)]
    print(tag_ids_seen)

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
        if tag_seen_in_group == False:
            tag_groups_seen.append(set([tag_id]))

    
    # Go through each pair ot tags seen
    
    for index1 in range(len(tag_ids_seen)):
        for index2 in range(index1+1, len(tag_ids_seen)):
            (tag_ids_seen[index1],tag_ids_seen[index2])
    #goes through all possible pairs, if 123 was a group, 1,2 1,3 2,3. 
            for gindex in range(len(tag_groups_seen)):
                if tag_ids_seen[index1] in tag_groups_seen[gindex]:
                    gindex1=gindex
                    break
    #uses above pairs to check for overlap of already seen, sets first group index or gindex1
            for gindex in range(len(tag_groups_seen)):
                if tag_ids_seen[index2] in tag_groups_seen[gindex]:
                    gindex2=gindex
                    break
        #uses pairs to check for overlap of already seen, sets gindex2. 
            if gindex1 == gindex2:
                continue
            #makes it so both gindexes cannot be the same. 

            tag_groups_seen[gindex1]=tag_groups_seen[gindex1].union(tag_groups_seen[gindex2])
            #puts them into the same set within already seen. 
            tag_groups_seen.pop(gindex2)
    print(tag_groups_seen)


def main():
    rospy.init_node('check_tags', anonymous=True)
    rospy.Subscriber('/apriltag_zed_front/apriltag_detection/tags', ApriltagArrayStamped, callback)

    # keep track of tags seen
    #  store a list of bool values, one per tag
    #     initialize the list to false for each tag
    for _ in range(NUMBER_OF_TAGS + 1):
        tags_seen.append(False)

    rospy.spin()
    
if __name__ == '__main__':
    main()