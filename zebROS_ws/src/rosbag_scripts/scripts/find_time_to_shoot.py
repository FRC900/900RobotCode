#! /bin/bash/python3

import rosbag
import csv
import sys

shooting_goal_sent = None
last_preshooter_switch = False


with open(csv_file_path, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)

    # Write header to the CSV file
    

    # Iterate through the messages in the specified topic
    for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages(['/frcrobot_rio/joint_states', '/shooting/shooting_server_2024/goal']):
        
        #mentioned something about me looking at the wrong place? mentioned someting about a limit swtich instead of trying to see if hte motors are actually moving etc

        if topic == '/shooting/shooting_server_2024/goal':
            shooting_goal_sent = t.to_sec()
            print("shooting goal is requested")

        elif topic == "/frcrobot_rio/joint_states":
             print("frcrobot joint activity")
             for i in range(len(msg.name)):
                  if msg.name[i] == "preshooter_limit_switch":
                        if msg.position[i] == 0 and shooting_goal_sent and last_preshooter_switch:
                            #print("got here")
                            print(f"{t.to_sec() - shooting_goal_sent}")
                            csv_writer.writerow([t.to_sec() - shooting_goal_sent])
                            shooting_goal_sent = None
                        last_preshooter_switch = msg.position[i]

                        


      

              

                
                

                

              
                
       

print(f"Data has been written to {csv_file_path}")
















