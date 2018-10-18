#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from behaviors.msg import *
# import all the actions

# define state Foo
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
    sm = smach.StateMachine(outcomes=['outcome4'])

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
                                transitions={'exit':'Init'})
        #actions!
        smach.StateMachine.add('PathToExchange', 
                                SimpleActionState('auto_loop_as',
                                            PathToExchangeAction),
                                transitions={'succeeded':'ScoreCube', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('PathToCube', 
                                SimpleActionState('auto_loop_as',
                                            PathToCubeAction),
                                transitions={'succeeded':'IntakeCube','aborted':'TestAtCenterC', 'preempted':'Exit'})
        smach.StateMachine.add('ScoreCube', 
                                SimpleActionState('auto_loop_as',
                                            ScoreCubeAction),
                                transitions={'succeeded':'TestHasCube', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('IntakeCube',
                                SimpleActionState('auto_loop_as',
                                            IntakeCubeAction),
                                transitions={'succeeded':'TestHasCube','aborted':'SpinOut', 'preempted':'Exit'})
        smach.StateMachine.add('PathToCenterC',
                                SimpleActionState('auto_loop_as',
                                            PathToCenterCAction),
                                transitions={'succeeded':'TurnToCube','aborted':'PathToCube', 'preempted':'Exit'})
        smach.StateMachine.add('PathToCenterE',
                                SimpleActionState('auto_loop_as',
                                            PathToCenterEAction),
                                transitions={'succeeded':'TurnToExchange','aborted':'PathToExchange', 'preempted':'Exit'})
        smach.StateMachine.add('TurnToCube',
                                SimpleActionState('auto_loop_as',
                                            TurnToCubeAction),
                                transitions={'succeeded':'PathToCube', 'aborted':'Party', 'preempted':'Exit'})
        smach.StateMachine.add('TurnToExchange',
                                SimpleActionState('auto_loop_as',
                                            TurnToExchangeAction),
                                transitions={'succeeded':'PathToExchange', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('SpinOut',
                                SimpleActionState('auto_loop_as',
                                           SpinOutAction),
                                transitions={'succeeded':'TestHasCube', 'aborted':'Exit', 'preempted':'Exit'})
        smach.StateMachine.add('Party',
                                SimpleActionState('auto_loop_as',
                                            PartyAction),
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
