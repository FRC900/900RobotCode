#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from behaviors.msg import *
from path_to_goal.msg import *
from actionlib_msgs.msg import GoalStatus

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

    def execute(self, userdata):
        if True:#line_break_sensor:
            return 'testTrue'
        else:
            return 'testFalse'

class TestAtCenterC(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['testTrue', 'testFalse'])

    def execute(self, userdata):
        if True: #check for center. LIDAR?
            return 'testTrue'
        else:
            return 'testFalse'
class TestAtCenterE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['testTrue', 'testFalse'])

    def execute(self, userdata):
        if True: #check for center. LIDAR?
            return 'testTrue'
        else:
            return 'testFalse'

class TestSeesCubes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['testTrue', 'testFalse'])

    def execute(self, userdata):
        if True:
            return 'testTrue'
        else:
            return 'testFalse'

successType = 0;
class MultipleSuccesses(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success0', 'success1'])

    def execute(self, userdata):
        print("MultipleSuccesses, successType")
        print(successType)
        if successType == 0:
            return 'success0'
        else:
            return 'success1'



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
                                transitions={'success':'ScoreCube','failure':'Exit'})
        smach.StateMachine.add('TestHasCube', TestHasCube(), 
                                transitions={'testTrue':'TestAtCenterE', 'testFalse':'TestAtCenterC'})
        smach.StateMachine.add('TestCollectedCube', MultipleSuccesses(),
                                transitions={'success0':'TestAtCenterE', 'success1':'SpinOut'})
        smach.StateMachine.add('TestAtCenterC', TestAtCenterC(),
                transitions={'testTrue':'TurnToCube', 'testFalse':'PathToCenterC'})
        smach.StateMachine.add('TestAtCenterE', TestAtCenterE(),
                transitions={'testTrue':'TurnToExchange', 'testFalse':'PathToCenterE'})
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
                                SimpleActionState('path_server',
                                            PathAction, goal=goalPathToExchange),
                                transitions={'succeeded':'ScoreCube', 'aborted':'Exit', 'preempted':'Exit'})
        goalPathToCube = PathGoal()
        goalPathToCube.goal_index = 2
        goalPathToCube.x = 0
        goalPathToCube.y = 0
        goalPathToCube.rotation = 0
        goalPathToCube.time_to_run = 50
        smach.StateMachine.add('PathToCube', 
                                SimpleActionState('path_server',
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
                                SimpleActionState('arm_server',
                                            ArmAction,goal=goalArmE),
                                transitions={'succeeded':'TestHasCube', 'aborted':'Exit', 'preempted':'Exit'})
        goalArmI = ArmGoal()
        goalArmI.arm_position = 0
        goalArmI.intake_cube = True
        goalArmI.intake_timeout = 10
        smach.StateMachine.add('IntakeCube',
                                SimpleActionState('arm_server',
                                            ArmAction,goal=goalArmI,result_cb=test_callback),
                                transitions={'succeeded':'TestCollectedCube','aborted':'Exit', 'preempted':'Exit'})
        goalPTCC = PathToCenterGoal()
        goalPTCC.usingCubeCenter = True
        smach.StateMachine.add('PathToCenterC',
                                SimpleActionState('pathToCenter_as',
                                            PathToCenterAction,goal=goalPTCC),
                                transitions={'succeeded':'TurnToCube','aborted':'Exit','preempted':'Exit'})
        goalPTCE = PathToCenterGoal()
        goalPTCE.usingCubeCenter = False
        smach.StateMachine.add('PathToCenterE',
                                SimpleActionState('pathToCenter_as',
                                            PathToCenterAction,goal=goalPTCE),
                                transitions={'succeeded':'TurnToExchange','aborted':'Exit','preempted':'Exit'})
        goalTurnCube = PathGoal()
        goalTurnCube.goal_index = 0
        goalTurnCube.x = 0
        goalTurnCube.y = 0
        goalTurnCube.rotation = 90 #essentially snap to angle; rework
        goalTurnCube.time_to_run = 50
        smach.StateMachine.add('TurnToCube',
                                SimpleActionState('path_server',
                                            PathAction, goal=goalTurnCube),
                                transitions={'succeeded':'TestSeesCubes', 'aborted':'Exit', 'preempted':'Exit'})
        goalTurnExchange = PathGoal()
        goalTurnExchange.goal_index = 0
        goalTurnExchange.x = 0
        goalTurnExchange.y = 0
        goalTurnExchange.rotation = 90 #essentially snap to angle; rework
        goalTurnExchange.time_to_run = 50
        smach.StateMachine.add('TurnToExchange',
                                SimpleActionState('path_server',
                                            PathAction, goal=goalTurnExchange),
                                transitions={'succeeded':'PathToExchange', 'aborted':'Exit', 'preempted':'Exit'})
        #define a goal for ejecting cubes
        goalIntake = IntakeGoal()
        goalIntake.intake_cube = False #because ejecting cube
        goalIntake.intake_timeout = 5
        smach.StateMachine.add('SpinOut',
                                SimpleActionState('intake_server',
                                            IntakeAction, goal=goalIntake),
                                transitions={'succeeded':'IntakeCube', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('Party',
                                SimpleActionState('party_as',
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
