# Take in a rosbag as a command line argument
# Analyze the rosbag, and for each time an apriltag detection is received,
# print out the velocity (/frcrobot_jetson/swerve_drive_controller/cmd_vel_out)

import rosbag
import sys
import os
import rospy
bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

last_vel = None
last_tf = None
preshooter_limit = 1
started = False
times_shot = 0

cmd_vel_threshold = 0.2
last_stopped = 0

'''
Shot! Time to align: 0.016313403

Shot! Time to align: 2.240238032
Shot! Time to align: 1.6411924450000002
Shot! Time to align: 2.067901171
Shot! Time to align: 1.070333666
Shot! Time to align: 1.098450487
Shot! Time to align: 2.379960855
Shot! Time to align: 1.749279895
Shot! Time to align: 2.00945089
Shot! Time to align: 1.849438493
Shot! Time to align: 1.288832633
Shot! Time to align: 1.5999168510000001
'''
start_align_time = rospy.Time()
# i added it at the end :) (at least for pivot)
for topic, msg, t in bag.read_messages(topics=["/shooting/shooting_server_2024/goal", "/shooter/set_shooter_speed/feedback", "/rosout", "/frcrobot_rio/joint_states", "/align_and_shoot/align_and_shoot_2024/goal", "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", "/shooter/set_shooter_pivot/feedback", "/shooter/set_shooter_pivot/goal"]):
    if topic == "/align_and_shoot/align_and_shoot_2024/goal":
        print(f"{t.to_sec()}: align and shoot goal sent")
        # print current time and that we received a goal
        start_time = t
        started = True 
        #print(f"Starting align at time: {t}")

    elif topic == "/shooting/shooting_server_2024/goal":
        print(f"{t.to_sec()}: shooting goal sent with distance {msg.goal.distance}")
        
    elif topic == "/frcrobot_rio/joint_states" and started:
        if preshooter_limit == 1 and not msg.position[msg.name.index("preshooter_limit_switch")]:
            print(f"Shot! Time to align: {(t - start_time).to_sec()}")
            print(f"Time to rotate: {(t - start_align_time).to_sec()}")
            print("\n\n\n")
            times_shot += 1
        preshooter_limit = msg.position[msg.name.index("preshooter_limit_switch")]
    elif topic == "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out":
        pass
    elif topic == "/shooter/set_shooter_pivot/feedback":
        # actionlib internals are annyoing
        # so the pivot is suspect 
        # why is it not already there 
        # i am confused about that as well
        # it appears to move at the end
        # we should check pivot goal # yeah lets see what gets sent, I am going to see if its obvious from code
        if msg.feedback.is_at_pivot_position:
            # uhh that definitely exists?
            print(f"Shooter pivot is at position at time: {t.to_sec()}")
    elif topic == "/shooter/set_shooter_pivot/goal":
        print(f"{t.to_sec()}: pivot position set to {msg.goal.pivot_position}")
    elif topic == "/rosout":
        # if message from /align_and_shoot/align_and_shoot_2024, print out the message and the time it was published
        if msg.name == "/align_to_speaker/align_to_speaker_2024" or msg.name == "/align_and_shoot/align_and_shoot_2024" or msg.name == "/shooter/shooter_server_2024" or msg.name == "/shooter/shooter_pivot_server_2024":
            print(f"{t.to_sec()}: {msg.msg}") # out-copiloted you lol :) :)
            if "Align to speaker " in msg.msg:
                # wait this is the robot relative angle
                # if you want field relative i think we echo /teleop/orientation_command?
                # well its just for getting aligning time so I think this is ok? 
                # oh ok
                start_align_time = t

print(f"Shot {times_shot} times")



    # wait so this seems bad right, why do we send to like 0.8 and then 0.5 
    # i agree
    # we do lower it for intaking
    # why are we intaking when shooting lmao
        # so the right position here is 0.8? or is it 0.5?
    # but i'd think shooting preparation when aligning gets it to the right position?
        # should be the second one
    # does the second shot look good? i think it does from the logs but have to check video
        # we make the last shot in the video
        # oh first shot is diverter messed up 
        # like the 0.01 one? I think so, its what happens first, then we make the shot
        
    # so what i am noticing is that the distance sent after aligning is different than the distance sent when starting to align
    # which is why we are moving the pivot to different locations
    # idea: dynamically checking distance in the shooting server or something?
        # yeah that seems like a good place to start, but also I wonder why we are wrong at the start
    # hmm idk
        # relocalization? i don't feel like it's that drastic though
        # do we know who sends these pivot commands
        # should be shooting server

    # ok i think what we need is a csv format
    # total time, time spent aligning, time spent pivoting, time spent spinning up, time after all those before we shoot
    # what do you think?
        # yeah seems good I think we are not like setting everything to run in parallel and waiting too early but idk yet
    # huh ok
    # i need to do a research in engineering thingy lol (it's due tomorrow)
    # but i'll be back
        # ok I see stuff, we are waiting for align and THEN waiting for shooting
        # i think we need to do that though since we don't want to shoot while aligning
        # we are setting up the shooter while aligning so theoretically the wait for shooting should be basically instant
        # except when we relocalize
        # ooh hm