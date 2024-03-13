#!/usr/bin/env python3

# Importing stuff, self-explanatory, but I'll explain it anyway because I'm bored :D
import rospy # If you don't know what this is, either you're on mechanical or VERY behind
import actionlib # Lets us do server stuff
from behavior_actions.msg import Arm2024Action, Arm2024Goal, Arm2024Feedback, Arm2024Result # The ".msg" of action servers
from std_msgs.msg import Float64 # Getting the Float64 type from std_msgs
from talon_state_msgs.msg import TalonFXProState # So we can see the position of the motor
from controllers_2024_msgs.srv import ShooterPivotSrv, ShooterPivotSrvRequest

class ArmAction(): # Creates ArmAction class
    # Defining our feedback and result stuff (Don't ask why there's an underscore at the front because I don't know either)
    _feedback = Arm2024Feedback()
    _result = Arm2024Result()

    def __init__(self, name): # "init"iate the variables. 
        self.rot_pos = 0.0
        self.rot_pos_goal = Float64() # Set the goal of rot_pos (rotation position) to a Float64
        self.sub = rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.callback) # Setup a subscriber to 'grab' the data in TalonFXProState (The state of the motors)
        # A bunch of parameters from the 2024_arm_control.yaml file so it's easier to change then when mechanical gives us a robot
        self.diverter_position = rospy.get_param("diverter_position")
        self.amp_position = rospy.get_param("amp_position")
        self.trap_position = rospy.get_param("trap_position")
        # Set up a publisher to publish stuff to the jetson
        #self.pub = rospy.Publisher('/frcrobot_jetson/arm_controller/command', Float64, queue_size=1)
        self.arm_client = rospy.ServiceProxy("/frcrobot_jetson/arm_controller/shooter_pivot_service", ShooterPivotSrv)

        self._action_name = name # Give it a name, used as a namespace (For people confuse [like me :D], a namespace is pretty much a dictionary that stores different stuff)
        self._as = actionlib.SimpleActionServer(self._action_name, Arm2024Action, execute_cb=self.execute_cb, auto_start = False) # Create the Action Server, and set autostart to False bc ros says so
        self._as.start() # "Start up" the server
        self.arm_index = None # Setting up a the arm_index variable to None so I can use it later.
        self.epsilon = rospy.get_param("tolerance") # This is the tolerance that we'll allow (Don't ask why its called epsilon, because I don't know either.)

    # The callback function so our subscriber knows what to do with the data it's pulling.
    def callback(self, data):
        if self.arm_index == None: # See how we start using arm_index here? It'll be more important why later
            for i in range(len(data.name)): # Starts searching through the names of the data
                if data.name[i] == "arm_joint": # Pretty much looking for the name of the motor that controls the arm (We named it 'arm' lol)
                    self.arm_index == i # Optimization purposes, so that the code doesn't have to look through everything for 'arm' more than once.
                    self.rot_pos = data.position[i] # Finds the proper position for the arm (The index of the position value will be the same as the index of the arm)
                    break
        else:
            self.rot_pos = data.position[self.arm_index] # Here, we use that optimization. Now, the code doesn't have to run the loop every time it tries to move the arm :D
    
    # The callback function for out action server so that it knows what to do with the data it's using (This is pretty much the main code lol)
    def execute_cb(self, goal):
        # Quality of life variables
        r = rospy.Rate(20) # Sets the rate at which ros runs the code to 20 (I think the unit is Hertz)
        success = True # Legibility, you'll get it later
        start_rot_pos = self.rot_pos # Sets the starting position to the first value of the motor


        # Set the motor's position goal to whatever it needs to be for said path.
        if goal.path == goal.DIVERTER: # If the motor's path is entered to be the diverter
            self.rot_pos_goal.data = self.diverter_position # The set the data of rot_pos_goal (rotation position goal) to the value of self.diverter_position. The reason why we have .data at the end is so python can use it in calculations. Remember when we set this to a Float64? Yea, python can't compute with that :/
            rospy.loginfo(f"{self._action_name}: Executing. Moving arm to DIVERTER - {self.rot_pos_goal.data}") # This will give info to the person running the server of whats going on

        # The rest is self-explanatory
        elif goal.path == goal.AMP:
            self.rot_pos_goal.data = self.amp_position
            rospy.loginfo(f"{self._action_name}: Executing. Moving arm to AMP - {self.rot_pos_goal.data}")
        
        elif goal.path == goal.TRAP:
            self.rot_pos_goal.data = self.trap_position
            rospy.loginfo(f"{self._action_name}: Executing. Moving arm to TRAP - {self.rot_pos_goal.data}")

        # self.pub.publish(self.rot_pos_goal) # Publish the position goal to the jetson.
        self.arm_client.call(ShooterPivotSrvRequest(self.rot_pos_goal.data))

        while success: # We set success = True, so this just makes it a fancy "while True" loop
            if rospy.is_shutdown(): # Check if we ^C'd the program, and if we did then it'll break the loop.
                success = False # Breaks the loop by setting success equal to False (See why I put success instead of true now?)
            self._feedback.percent_complete = ((self.rot_pos - start_rot_pos)/(self.rot_pos_goal.data - start_rot_pos)) * 100 # Let the feedback be equal to the position of the motor divided by where the motor wants to be. Multiply by 100 to make it a percent.
            
            self._as.publish_feedback(self._feedback) # Put the feedback so that it shows up on axclient
            
            # Check if the goal is preempted (canceled) and end the action if it is
            if self._as.is_preempt_requested():
                rospy.loginfo(f'{self._action_name}: Preempted')
                self._as.set_preempted()
                success = False # Break the loop because we didn't succeed :(
            # Set the equation for the percent difference, since we won't ever be 100% accurate, we can have some tolerance :D
            percent_difference = ((abs(self.rot_pos - self.rot_pos_goal.data))/((self.rot_pos + self.rot_pos_goal.data)/2)) * 100
            if (percent_difference < self.epsilon): # If the percent difference is less than the set tolerance, then we can start the success prompt
                self._result.success = True # You did it 🥳
                self._feedback.percent_complete = 100.00 # Visual purposes; e.g. so that axclient doesn't show like "99.995%" and it'll just show 100% instead
                self._as.publish_feedback(self._feedback) # Publish that 100%
                rospy.loginfo(f"{self._action_name}: Succeeded") # Yayyyy
                self._as.set_succeeded(self._result) # Set the result as succeeded
                break
            r.sleep() # Slows down the loop to run at a specified speed

# Run everything
if __name__ == '__main__': # I think this is if the name of the server is the same as the name of the server we want to run. Probably prevents accidental activation
    rospy.init_node('2024_move_arm_server', anonymous=True) # Initiates the node (wow really?)
    server = ArmAction(rospy.get_name()) # Pretty much ros finds the name of the code automagically
    rospy.spin() # SpInNnNn - A fancier "while: True" loop because now ros controls the speed of the loop :O