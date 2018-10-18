#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from behaviors.msg import PathToExchange
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
                                            PathToExchange),
                                transitions={'success':'ScoreCube'})
        smach.StateMachine.add('PathToCube', 
                                SimpleActionState('auto_loop_as',
                                            PathToCube),
                                transitions={'success':'IntakeCube','failure':'TestAtCenterC'})
        smach.StateMachine.add('ScoreCube', 
                                SimpleActionState('auto_loop_as',
                                            ScoreCube),
                                transitions={'success':'TestHasCube'})
        smach.StateMachine.add('IntakeCube',
                                SimpleActionState('auto_loop_as',
                                            IntakeCube),
                                transitions={'success':'TestHasCube','failure':'SpinOut'})
        smach.StateMachine.add('PathToCenterC',
                                SimpleActionState('auto_loop_as',
                                            PathToCenterC),
                                transitions={'success':'TurnToCube','seeEarly':'PathToCube'})
        smach.StateMachine.add('PathToCenterE',
                                SimpleActionState('auto_loop_as',
                                            PathToCenterE),
                                transitions={'success':'TurnToExchange','seeEarly':'PathToExchange'})
        smach.StateMachine.add('TurnToCube',
                                SimpleActionState('auto_loop_as',
                                            TurnToCube),
                                transitions={'success':'PathToCube','failure':'Party'})
        smach.StateMachine.add('TurnToExchange',
                                SimpleActionState('auto_loop_as',
                                            TurnToExchange),
                                transitions={'success':'PathToExchange'})
        smach.StateMachine.add('SpinOut',
                                SimpleActionState('auto_loop_as',
                                           SpinOut),
                                transitions={'success':'TestHasCube'})
        smach.StateMachine.add('Party',
                                SimpleActionState('auto_loop_as',
                                            Party),
                                transitions={'success':'PathToCube'})

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
