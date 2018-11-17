#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from behaviors.msg import *
from path_to_goal.msg import *
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        if(False):
            return 'failure'
        return 'success'


# define state Bar
class TestHasCube(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['testTrue', 'testFalse'])

        #set up subscriber to receive sensor data
        self.sub = rospy.Subscriber('/frcrobot/joint_states',JointState,self.callback)
        
        self.test_result = "default"  #initialize variable to store received msgs

    def callback(self,msg):
        sensor_index = 0
        for i in range(len(msg.position)):
            if msg.name[i] == "intake_line_break":
                sensor_index = i

        self.test_result = msg.position[sensor_index]

    def execute(self, userdata):
        rospy.loginfo("testhascube "+str(self.test_result))
        
        if self.test_result == 1.0: #line_break_sensor:
            return 'testTrue'
        else:
            return 'testFalse'

class TestAtCenterC(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['testTrue', 'testFalse'])

        #set up subscriber to read data
        #self.sub = rospy.Subscriber('topic_name',String,self.callback)

        self.test_result = "default"
    
    def callback(self,msg):
        self.test_result = msg

    def execute(self, userdata):
        if True: #check for center. LIDAR?
            return 'testTrue'
        else:
            return 'testFalse'
class TestAtCenterE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['testTrue', 'testFalse'])

        #set up subscriber to receive sensor data
        #self.sub = rospy.Subscriber('topic_name',String,self.callback)

        self.test_result = "default"
    
    def callback(self,msg):
        self.test_result = msg

    def execute(self, userdata):
        if True: #check for center. LIDAR?
            return 'testTrue'
        else:
            return 'testFalse'

class TestSeesCubes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['testTrue', 'testFalse'])
        
        #set up subscriber to receive data
        #self.sub = rospy.Subscriber('topic_name',String,self.callback)

        self.test_result = "default"
    
    def callback(self,msg):
        self.test_result = msg

    def execute(self, userdata):
        if True:
            return 'testTrue'
        else:
            return 'testFalse'

successType = 0;
class TestArmStuck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['testTrue', 'testFalse'])
    def execute(self, userdata):
        print("MultipleSuccesses, successType")
        print(successType)
        if successType == 0: #0 means stuck, 1 means fine
            return 'testTrue'
        else:
            return 'testFalse'



class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit'])

    def execute(self, userdata):
        rospy.loginfo('EXITED AUTONOMOUS')
        return 'exit'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exited'])

    # Open the container
    with sm:
        #not actions, logic states
        smach.StateMachine.add('Init', Init(), 
                                transitions={'success':'TestHasCube','failure':'Exit'})
        smach.StateMachine.add('TestHasCube', TestHasCube(), 
                                transitions={'testTrue':'TestAtCenterE', 'testFalse':'TestAtCenterC'})
        smach.StateMachine.add('TestCollectedCube', TestHasCube(),
                transitions={'testTrue':'TestAtCenterE', 'testFalse':'SpinOut'}) #TestArmStuck
        smach.StateMachine.add('TestAtCenterC', TestAtCenterC(),
                transitions={'testTrue':'TurnToCube', 'testFalse':'PathToCenterC'})
        smach.StateMachine.add('TestAtCenterE', TestAtCenterE(),
                transitions={'testTrue':'TurnToExchange', 'testFalse':'PathToCenterE'})
        smach.StateMachine.add('TestArmStuck', TestArmStuck(),
                transitions={'testTrue':'ResetArmPos', 'testFalse': 'SpinOut'})
        smach.StateMachine.add('Exit', Exit(),               
                                transitions={'exit':'exited'})
        smach.StateMachine.add('TestSeesCubes', TestSeesCubes(),
                transitions={'testTrue':'PathToCube', 'testFalse':'Party'})
        #actions
        goalPathToExchange = PathGoal()
        goalPathToExchange.goal_index = 2
        goalPathToExchange.x = 0
        goalPathToExchange.y = 0
        goalPathToExchange.rotation = 0
        goalPathToExchange.time_to_run = 50
        smach.StateMachine.add('PathToExchange', 
                                SimpleActionState('/frcrobot/path_server',
                                            PathAction, goal=goalPathToExchange),
                                transitions={'succeeded':'ScoreCube', 'aborted':'Exit', 'preempted':'Exit'})
        goalPathToCube = PathGoal()
        goalPathToCube.goal_index = 1
        goalPathToCube.x = 0
        goalPathToCube.y = 0
        goalPathToCube.rotation = 0
        goalPathToCube.time_to_run = 50
        smach.StateMachine.add('PathToCube', 
                                SimpleActionState('/frcrobot/path_server',
                                            PathAction, goal=goalPathToCube),
                                transitions={'succeeded':'IntakeCube','aborted':'Exit', 'preempted':'Exit'})
        
        def test_callback(userdata, status, result):
            global successType
            print("status")
            print(status)
            if status == GoalStatus.SUCCEEDED:
                print("result.is_true")
                print(result.is_true)
                if(result.is_true == True):
                    successType = 0
                else:
                    successType = 1
                
                print("successType")
                print(successType)
                return 'succeeded'

        goalArmE = ArmGoal()
        goalArmE.arm_position = 2
        goalArmE.intake_cube = False
        goalArmE.intake_timeout = 10
        smach.StateMachine.add('ScoreCube', 
                                SimpleActionState('/frcrobot/arm_server',
                                            ArmAction,goal=goalArmE),
                                transitions={'succeeded':'TestHasCube', 'aborted':'Exit', 'preempted':'Exit'})
        goalArmI = ArmGoal()
        goalArmI.arm_position = 0
        goalArmI.intake_cube = True
        goalArmI.intake_timeout = 10
        smach.StateMachine.add('IntakeCube',
                                SimpleActionState('/frcrobot/arm_server',
                                            ArmAction,goal=goalArmI),
                                transitions={'succeeded':'TestCollectedCube','aborted':'Exit', 'preempted':'Exit'})
        #below- this should be forearm goal, not arm goal, fix!!!
        goalArmR = ArmGoal()
        goalArmR.arm_position = 1
        goalArmR.intake_cube = True
        goalArmR.intake_timeout = 2
        smach.StateMachine.add('ResetArmPos',
                                SimpleActionState('/frcrobot/arm_server',
                                            ArmAction,goal=goalArmR),
                                transitions={'succeeded':'MoveRobotBack','aborted':'Exit','preempted':'Exit'})
        goalMoveBack = PathGoal()
        goalMoveBack.goal_index = 0
        goalMoveBack.x = 0
        goalMoveBack.y = -0.5
        goalMoveBack.rotation = 90
        goalMoveBack.time_to_run = 50
        smach.StateMachine.add('MoveRobotBack',
                                SimpleActionState('/frcrobot/path_server',
                                            PathAction, goal=goalMoveBack),
                                transitions={'succeeded':'TestSeesCubes','aborted':'Exit','preempted':'Exit'})
        goalPTCC = PathGoal()
        goalPTCC.goal_index = 3
        goalPTCC.x = 0
        goalPTCC.y = 0
        goalPTCC.rotation = 90
        goalPTCC.time_to_run = 50
        smach.StateMachine.add('PathToCenterC',
                                SimpleActionState('/frcrobot/path_server',
                                            PathAction,goal=goalPTCC),
                                transitions={'succeeded':'TurnToCube','aborted':'Exit','preempted':'Exit'})
        goalPTCE = PathGoal()
        goalPTCE.goal_index = 3
        goalPTCE.x = 0
        goalPTCE.y = 0
        goalPTCE.rotation = 90
        goalPTCE.time_to_run = 50
        smach.StateMachine.add('PathToCenterE',
                                SimpleActionState('/frcrobot/path_server',
                                            PathAction,goal=goalPTCE),
                                transitions={'succeeded':'TurnToExchange','aborted':'Exit','preempted':'Exit'})
        goalTurnCube = PathGoal()
        goalTurnCube.goal_index = 0
        goalTurnCube.x = 0
        goalTurnCube.y = 0
        goalTurnCube.rotation = 90 #essentially snap to angle; rework
        goalTurnCube.time_to_run = 50
        smach.StateMachine.add('TurnToCube',
                                SimpleActionState('/frcrobot/path_server',
                                            PathAction, goal=goalTurnCube),
                                transitions={'succeeded':'TestSeesCubes', 'aborted':'Exit', 'preempted':'Exit'})
        goalTurnExchange = PathGoal()
        goalTurnExchange.goal_index = 0
        goalTurnExchange.x = 0
        goalTurnExchange.y = 0
        goalTurnExchange.rotation = 90 #essentially snap to angle; rework
        goalTurnExchange.time_to_run = 50
        smach.StateMachine.add('TurnToExchange',
                                SimpleActionState('/frcrobot/path_server',
                                            PathAction, goal=goalTurnExchange),
                                transitions={'succeeded':'PathToExchange', 'aborted':'Exit', 'preempted':'Exit'})
        #define a goal for ejecting cubes
        goalIntake = IntakeGoal()
        goalIntake.intake_cube = False #because ejecting cube
        goalIntake.timeout = 5
        smach.StateMachine.add('SpinOut',
                                SimpleActionState('/frcrobot/intake_server',
                                            IntakeAction, goal=goalIntake),
                                transitions={'succeeded':'PathToCube', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('Party',
                                SimpleActionState('/frcrobot/party_as',
                                            SingleExitAction),
                                transitions={'succeeded':'Exit', 'aborted':'Exit', 'preempted':'Exit'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
