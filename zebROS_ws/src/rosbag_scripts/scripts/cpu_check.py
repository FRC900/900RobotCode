#!/usr/bin/env python3

import sys
import rosbag

bag = rosbag.Bag(sys.argv[1])
cpu_topics = [t for t in bag.get_type_and_topic_info()[1].keys() if (('cpu_monitor' in t) and (t.endswith('cpu')))]

highest_cpu = dict([(topic, 0) for topic in cpu_topics])

for topic, msg, t in bag.read_messages(topics=cpu_topics):
    if msg.data > highest_cpu[topic]:
        highest_cpu[topic] = msg.data

print("Highest CPU usage:")
sum = 0

# Print the highest CPU usage for each topic sorted by value
highest_cpu = dict(sorted(highest_cpu.items(), key=lambda item: item[1], reverse=True))
for topic, cpu in highest_cpu.items():
    print(f"{topic}: {cpu}")

# for topic, cpu in highest_cpu.items():
#     print(f"{topic}: {cpu}")
#     if ('frcrobot_jetson' in topic):
#         sum += cpu

# print(f"Total CPU usage: {sum}")
