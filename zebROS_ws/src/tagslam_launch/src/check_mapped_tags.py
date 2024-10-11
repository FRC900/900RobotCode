from scipy.spatial.transform import Rotation
import yaml
import pprint
import math

MAPPED_TAGS_PATH = "/home/ubuntu/900RobotCode/zebROS_ws/src/tagslam_launch/y2024_comp/wake_mapped_tags.yaml"
DEFAULT_TAGS_PATH = "/home/ubuntu/900RobotCode/zebROS_ws/src/tagslam_launch/y2024_comp/backup_tag_map.yaml"

# load both yaml's
with open(MAPPED_TAGS_PATH, "r") as f:
    mapped_tags = yaml.load(f, yaml.FullLoader)

with open(DEFAULT_TAGS_PATH, "r") as f:
    default_tags = yaml.load(f, yaml.FullLoader)

mapped_tags = mapped_tags["bodies"][0]["zebracorn_labs"]["tags"]
default_tags = default_tags["bodies"][0]["zebracorn_labs"]["tags"]

# {k: v for k, v in sorted(mapped_tags.items(), key=lambda item: item[0])}
pprint.pprint(mapped_tags)
default_tags_sorted = {}
mapped_tags_sorted = {}

while len(mapped_tags_sorted) != 16:
    print(len(mapped_tags_sorted))
    for tag in mapped_tags:
        if tag["id"] == len(mapped_tags_sorted) + 1:
            mapped_tags_sorted[tag["id"]] = tag

while len(default_tags_sorted) != 16:
    print(len(default_tags_sorted))
    for tag in default_tags:
        if tag["id"] == len(default_tags_sorted) + 1:
            default_tags_sorted[tag["id"]] = tag

print(mapped_tags_sorted)
print(default_tags_sorted)
# tags 1-16
for i in range(1, 17):
    print(mapped_tags_sorted[i]["pose"]["position"], default_tags_sorted[i]["pose"]["position"])
    map_tag_pos = mapped_tags_sorted[i]["pose"]["position"]
    default_tag_pos = default_tags_sorted[i]["pose"]["position"]
    distance = math.hypot(map_tag_pos["x"], map_tag_pos["y"], map_tag_pos["z"]) - math.hypot(default_tag_pos["x"], default_tag_pos["y"], default_tag_pos["z"])
    print(f"Tag ID {i} off by {distance}")