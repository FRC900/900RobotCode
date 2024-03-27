#! /bin/bash/python3

import rosbag
import csv
import sys

# Define the topic you want to read
topic_talon = '/frcrobot_jetson/talonfxpro_states'
shoot_request = '/shooting/shooting_server_2024/goal'

shoot_time = 0
velocity_time = 0
time_delta = 0

ticker_one = 0
index = 0
shoot_request_bool = False
topic_talon_bool = False
csv_file_path = 'output.csv'
with open(csv_file_path, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)

    # Write header to the CSV file
    csv_writer.writerow(['time_delta', 'shoot_time', 'velocity_time'])

    # Iterate through the messages in the specified topic
    for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages(['/frcrobot_jetson/talonfxpro_states', '/shooting/shooting_server_2024/goal']):
        


        if topic == '/shooting/shooting_server_2024/goal':
            shoot_time = t.to_sec()
            shoot_request_bool = True


        if topic == '/frcrobot_jetson/talonfxpro_states':
            index = msg.name.index('bottom_left_shooter_joint')
            velocity = msg.control_velocity[index]
            if velocity > 100:
                velocity_time = t.to_sec()
            topic_talon_bool = True


        if ((shoot_request_bool == True) and (topic_talon_bool == True)):
            #Write data to the CSV file
            if shoot_time < velocity_time:
                time_delta = velocity_time - shoot_time

              

                #print(t.to_sec())
                
                csv_writer.writerow([time_delta, shoot_time, velocity_time])

                

                shoot_time = -1
                velocity_time = -1
                time_delta = -1
                shoot_request_bool = False
                topic_talon_bool = False
                
       

print(f"Data has been written to {csv_file_path}")














