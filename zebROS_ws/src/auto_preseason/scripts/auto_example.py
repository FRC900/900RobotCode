#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from behaviors.msg import *
from path_to_goal.msg import *

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
        smach.StateMachine.add('TestAtCenterC', TestAtCenterC(),
                transitions={'testTrue':'TurnToCube', 'testFalse':'PathToCenterC'})
        smach.StateMachine.add('TestAtCenterE', TestAtCenterE(),
                transitions={'testTrue':'TurnToExchange', 'testFalse':'PathToCenterE'})
        smach.StateMachine.add('Exit', Exit(),
                                transitions={'exit':'exited'})
        #actions!
        goalPathToExchange = PathActionGoal()
        goalPathToExchange.goal.goal_index = 2
        goalPathToExchange.goal.x = 0
        goalPathToExchange.goal.y = 0
        goalPathToExchange.goal.rotation = 0
        goalPathToExchange.goal.time_to_run = 50
        smach.StateMachine.add('PathToExchange', 
                                SimpleActionState('path_server',
                                            PathAction, goal=goalPathToExchange),
                                transitions={'succeeded':'ScoreCube', 'aborted':'Exit', 'preempted':'Exit'})
        goalPathToCube = PathActionGoal()
        goalPathToCube.goal.goal_index = 2
        goalPathToCube.goal.x = 0
        goalPathToCube.goal.y = 0
        goalPathToCube.goal.rotation = 0
        goalPathToCube.goal.time_to_run = 50
        smach.StateMachine.add('PathToCube', 
                                SimpleActionState('path_server',
                                            PathAction, goal=goalPathToCube),
                                transitions={'succeeded':'IntakeCube','aborted':'TestAtCenterC', 'preempted':'Exit'})
        goalArmE = ArmActionGoal()
        goalArmE.goal.arm_position = 2
        goalArmE.goal.intake_cube = False
        goalArmE.goal.intake_timeout = 10
        smach.StateMachine.add('ScoreCube', 
                                SimpleActionState('arm_server',
                                            ArmAction,goal=goalArmE),
                                transitions={'succeeded':'TestHasCube', 'aborted':'Exit', 'preempted':'Exit'})
        goalArmI = ArmActionGoal()
        goalArmI.goal.arm_position = 0
        goalArmI.goal.intake_cube = True
        goalArmI.goal.intake_timeout = 10
        smach.StateMachine.add('IntakeCube',
                                SimpleActionState('arm_server',
                                            ArmAction,goal=goalArmI),
        transitions={'succeeded':'TestHasCube','aborted':'SpinOut', 'preempted':'Exit'})
        goalPTCC = PathToCenterActionGoal()
        goalPTCC.goal.usingCubeCenter = True
        smach.StateMachine.add('PathToCenterC',
                                SimpleActionState('pathToCenter_as',
                                            PathToCenterAction,goal=goalPTCC),
                                transitions={'succeeded':'TurnToCube','aborted':'PathToCube', 'preempted':'Exit'})
        goalPTCE = PathToCenterActionGoal()
        goalPTCE.goal.usingCubeCenter = False
        smach.StateMachine.add('PathToCenterE',
                                SimpleActionState('pathToCenter_as',
                                            PathToCenterAction,goal=goalPTCE),
                                transitions={'succeeded':'TurnToExchange','aborted':'PathToExchange', 'preempted':'Exit'})
        goalTurnCube = PathActionGoal()
        goalTurnCube.goal.goal_index = 0
        goalTurnCube.goal.x = 0
        goalTurnCube.goal.y = 0
        goalTurnCube.goal.rotation = 90 #essentially snap to angle; rework
        goalTurnCube.goal.time_to_run = 50
        smach.StateMachine.add('TurnToCube',
                                SimpleActionState('turnToCube_as',
                                            PathAction, goal=goalTurnCube),
                                transitions={'succeeded':'PathToCube', 'aborted':'Party', 'preempted':'Exit'})
        goalTurnExchange = PathActionGoal()
        goalTurnExchange.goal.goal_index = 0
        goalTurnExchange.goal.x = 0
        goalTurnExchange.goal.y = 0
        goalTurnExchange.goal.rotation = 90 #essentially snap to angle; rework
        goalTurnExchange.goal.time_to_run = 50
        smach.StateMachine.add('TurnToExchange',
                                SimpleActionState('path_server',
                                            PathAction, goal=goalTurnExchange),
                                transitions={'succeeded':'PathToExchange', 'aborted':'Exit', 'preempted':'Exit'})
        #define a goal for ejecting cubes
        goalIntake = IntakeActionGoal()
        goalIntake.goal.intake_cube = False #because ejecting cube
        goalIntake.goal.intake_timeout = 5
        smach.StateMachine.add('SpinOut',
                                SimpleActionState('intake_server',
                                            IntakeAction, goal=goalIntake),
                                transitions={'succeeded':'TestHasCube', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('Party',
                                SimpleActionState('party_as',
                                            SingleExitAction),
                                transitions={'succeeded':'PathToCube', 'aborted':'Exit', 'preempted':'Exit'})

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
