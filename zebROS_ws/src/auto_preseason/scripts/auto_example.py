#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from behaviors.msg import *

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
                                transitions={'success':'testHasCube','failure':'Exit'})
        smach.StateMachine.add('TestHasCube', TestHasCube(), 
                                transitions={'testTrue':'TestAtCenterE', 'testFalse':'TestAtCenterC'})
        smach.StateMachine.add('TestAtCenterC', TestAtCenterC(),
                transitions={'testTrue':'TurnToCube', 'testFalse':'PathToCenterC'})
        smach.StateMachine.add('TestAtCenterE', TestAtCenterE(),
                transitions={'testTrue':'TurnToExchange', 'testFalse':'PathToCenterE'})

        smach.StateMachine.add('Exit', Exit(),
                                transitions={'exit':'exited'})
        #actions!
        smach.StateMachine.add('PathToExchange', 
                                SimpleActionState('pathToExchange_as',
                                            SingleExitAction),
                                transitions={'succeeded':'ScoreCube', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('PathToCube', 
                                SimpleActionState('pathToCube_as',
                                            DualExitAction),
                                transitions={'succeeded':'IntakeCube','aborted':'TestAtCenterC', 'preempted':'Exit'})
        smach.StateMachine.add('ScoreCube', 
                                SimpleActionState('scoreCube_as',
                                            SingleExitAction),
                                transitions={'succeeded':'TestHasCube', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('IntakeCube',
                                SimpleActionState('intakeCube_as',
                                            DualExitAction),
                                transitions={'succeeded':'TestHasCube','aborted':'SpinOut', 'preempted':'Exit'})
        smach.StateMachine.add('PathToCenterC',
                                SimpleActionState('pathToCenter_as',
                                            PathToCenterAction,goal=True),
                                transitions={'succeeded':'TurnToCube','aborted':'PathToCube', 'preempted':'Exit'})
        smach.StateMachine.add('PathToCenterE',
                                SimpleActionState('pathToCenter_as',
                                            PathToCenterAction,goal=False),
                                transitions={'succeeded':'TurnToExchange','aborted':'PathToExchange', 'preempted':'Exit'})
        smach.StateMachine.add('TurnToCube',
                                SimpleActionState('turnToCube_as',
                                            DualExitAction),
                                transitions={'succeeded':'PathToCube', 'aborted':'Party', 'preempted':'Exit'})
        smach.StateMachine.add('TurnToExchange',
                                SimpleActionState('turnToExchange_as',
                                            SingleExitAction),
                                transitions={'succeeded':'PathToExchange', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('SpinOut',

                                SimpleActionState('spinOut_as',
                                            SpinOutAction, goal=5.0),
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
