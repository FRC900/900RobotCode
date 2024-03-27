#! /bin/bash/python3

import rosbag
import csv
import sys

shooting_goal_sent = None
last_preshooter_switch = False

list_of_bag_files = ['20240317_191904_NCASH_E11.bag', '20240317_182218_NCASH_E07.bag', '20240317_173126_NCASH_E02.bag', '20240317_150254_NCASH_Q52.bag', '20240317_143711_NCASH_Q49.bag', '20240317_132812_NCASH_Q43.bag', '20240316_223841_NCASH_Q40.bag', '20240316_215846_NCASH_Q36.bag', '20240316_210646_NCASH_Q30.bag', '20240316_203217_NCASH_Q26.bag', '20240316_194458_NCASH_Q21.bag', '20240316_184618_NCASH_Q15.bag', '20240316_181428_NCASH_Q12.bag', '20240316_163338_NCASH_Q08.bag', '20240316_155516_NCASH_Q04.bag']

csv_file_path = 'output.csv'
with open(csv_file_path, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)

    # Write header to the CSV file
    for i in list_of_bag_files:

        # Iterate through the messages in the specified topic
        for topic, msg, t in rosbag.Bag(i).read_messages(['/frcrobot_rio/joint_states', '/shooting/shooting_server_2024/goal']):
            
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
                                if ((t.to_sec() - shooting_goal_sent) > .1):
                                    csv_writer.writerow([t.to_sec() - shooting_goal_sent])
                                shooting_goal_sent = None
                            last_preshooter_switch = msg.position[i]

                        


      

              

                
                

                

              
                
       

print(f"Data has been written to {csv_file_path}")
















