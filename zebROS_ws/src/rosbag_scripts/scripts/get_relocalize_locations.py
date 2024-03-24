#!/usr/bin/env python3
# Create a CSV file with all x,y positions where TagSLAM relocalized, then use matplotlib to generate a heatmap
# TagSLAM relocalization is defined as a map -> frc_robot transform being published
import rosbag
import sys
import os

bagfile_names = sys.argv[1:]

locations = []
distances = []
tags = []

for bagfile_name in bagfile_names:
    print(bagfile_name)
    bag = rosbag.Bag(bagfile_name)

    last_vel = None
    last_tf = None

    last_tag_seen = None
    latest_front_tags = []
    latest_back_tags = []
    tag_timeout = 0.1

    for topic, msg, t in bag.read_messages(topics=["/tf", "/apriltag_zedx_back/tag_detection_world", "/apriltag_zedx_front/tag_detection_world"]):
        if topic == "/apriltag_zedx_back/tag_detection_world" or topic == "/apriltag_zedx_front/tag_detection_world":
            if len(msg.objects) > 0:
                last_tag_seen = msg.header.stamp
                exec(f"latest_{'front' if 'front' in topic else 'back'}_tags = list(map(lambda tag: (tag.id, (tag.location.x**2 + tag.location.y**2 + tag.location.z**2)**0.5), msg.objects))")
        if topic == "/tf":
            for tf in msg.transforms:
                if tf.header.frame_id == "map" and tf.child_frame_id == "frc_robot" and last_tag_seen is not None and tf.header.stamp.to_sec() - last_tag_seen.to_sec() < tag_timeout:
                    # print(f"{t.to_sec()},{tf.transform.translation.x},{tf.transform.translation.y}")
                    locations.append((tf.transform.translation.x, tf.transform.translation.y))
                    distances += list(map(lambda tag: tag[1], latest_front_tags + latest_back_tags))
                    tags.append((latest_front_tags, latest_back_tags))
                    latest_front_tags = []
                    latest_back_tags = []

# Save locations and tags to a CSV file
with open("relocalize_locations.csv", "w") as f:
    f.write("x,y,tag1,tag1Dist,tag2,tag2Dist,tag3,tag3Dist,tag4,tag4Dist,tag5,tag5Dist,tag6,tag6Dist\n")
    for j in range(len(locations)):
        loc = locations[j]
        f.write(f"{loc[0]},{loc[1]}")
        these_tags = tags[j][0] + tags[j][1]
        for i in range(6):
            if i < len(these_tags):
                f.write(f",{these_tags[i][0] if len(these_tags) > i else '0'},{these_tags[i][1] if len(these_tags) > i else '0'}")
            else:
                f.write(",0,0")
        f.write("\n")

# Generate heatmap

import matplotlib.pyplot as plt
import matplotlib.colors
import numpy as np

x = [loc[0] for loc in locations]
y = [loc[1] for loc in locations]

# Make the heatmap opacity 0.5, and overlay the map of the field on top of it
# The map is located at /home/ubuntu/2023RobotCode/zebROS_ws/src/controller_node/maps/crescendo_field.png

img = plt.imread("/home/ubuntu/2023RobotCode/zebROS_ws/src/controller_node/maps/crescendo_field.png")

plt.imshow(img, extent=[0, 16.54, 0, 8.21])
plt.scatter(x, y, alpha=0.5) # hist2d , bins=100, norm=matplotlib.colors.LogNorm(), cmap='Greys'
# Set bounds to 16.54 x 8.21
plt.xlim(0, 16.54)
plt.ylim(0, 8.21)
# plt.colorbar(hb)

# Add annotations to each point on hover that show the tags
# (motion_notify_event)
def hover(event):
    if event.inaxes == plt.gca():
        for i, (x, y) in enumerate(locations):
            if abs(event.xdata - x) < 0.1 and abs(event.ydata - y) < 0.1:
                plt.gca().set_title(f"{tags[i]}")
                plt.draw()
                return
    plt.gca().set_title("")
    plt.draw()

plt.gcf().canvas.mpl_connect('motion_notify_event', hover)

plt.show()

plt.savefig("relocalize_locations.png")

# Show histogram of tag distances
plt.hist(distances, bins=100)
plt.show()
plt.savefig("relocalize_distances.png")