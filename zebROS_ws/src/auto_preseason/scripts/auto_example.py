#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
# import all the actions

# define state Foo
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        return 'success'


# define state Bar
class CheckForCube(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['haz_cube', 'haz_no_cube'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        if True:#line_break_sensor:
            return 'haz_cube'
        else:
            return 'haz_no_cube'

class HazCube(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sees_exchange', 'sees_no_exchange'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        if True: #len(exchange_msg) > 0:
            return 'sees_exchange'
        else:
            return 'sees_no_exchange'
  
class HazNoCube(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sees_cube', 'sees_no_cube'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        if True: #len(cube_msg) > 0:
            return 'sees_cube'
        else:
            return 'sees_no_cube'

class CheckCenter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered_look_for_cube', 'centered_look_for_exchange', 'not_centered'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ExchangeCube')
        if True: #check for center. LIDAR?
            return 'centered'
        else:
            return 'not_centered'

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
                               transitions={'success':'CheckForCube'})
        smach.StateMachine.add('CheckForCube', CheckForCube(), 
                                transitions={'haz_cube':'HazCube', 'haz_no_cube':'HazNoCube'})
        smach.StateMachine.add('HazCube', HazCube(), 
                                transitions={'sees_exchange':'pathToExchange', 'sees_no_exchange':'CheckForCenter'})
        smach.StateMachine.add('HazNoCube', HazNoCube(), 
                                transitions={'sees_cube':'pathToCube', 'sees_no_cube':'CheckForCenter'})
        smach.StateMachine.add('CheckForCenter', CheckCenter(),
                transitions={'centered_look_for_cube':'TurnForCube', 'centered_look_for_exchange':'TurnForExchange', 'not_centered':'PathToCenter'})
        smach.StateMachine.add('Exit', Exit(),
                                transitions={'exit':'Init'})
        #actions!
        smach.StateMachine.add('PathToExchange', 
                                SimpleActionState('auto_loop_as',
                                            PathToExchange),
                                transitions={'success':'ExchangeCube'})
        smach.StateMachine.add('PathToCube', 
                                SimpleActionState('auto_loop_as',
                                            PathToCube),
                                transitions={'success':'IntakeCube'})
        smach.StateMachine.add('ExchangeCube', 
                                SimpleActionState('auto_loop_as',
                                            ExchangeCube),
                                transitions={'success':'CheckForCube'})
        smach.StateMachine.add('IntakeCube',
                                SimpleActionState('auto_loop_as',
                                            IntakeCube),
                                transitions={'success':'CheckForCube'})
        smach.StateMachine.add('PathToCenter',
                                SimpleActionState('auto_loop_as',
                                            PathToCenter),
                                transitions={'success':'CheckForCube'})
        smach.StateMachine.add('TurnToCube',
                                SimpleActionState('auto_loop_as',
                                            TurnToCube),
                                transitions={'success':'PathToCube'})
        smach.StateMachine.add('TurnToExchange',
                                SimpleActionState('auto_loop_as',
                                            TurnToExchange),
                                transitions={'success':'PathToExchange'})
        smach.StateMachine.add('SpinOut',
                                SimpleActionState('auto_loop_as',
                                            SpinOut),
                                transitions={'success':'CheckForCube'})
        smach.StateMachine.add('PARTY',
                                SimpleActionState('auto_loop_as',
                                            TurnToExchange),
                                transitions={'abort':'Init'})


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
