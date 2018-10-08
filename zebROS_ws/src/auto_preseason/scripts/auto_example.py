#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

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

class PathToExchange(smach.State): # ACTIONLIB
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PathToExchange')
        ## execute actionlib here
        return 'success'


class ExchangeCube(smach.State): # ACTIONLIB
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ExchangeCube')
        ## execute actionlib here
        return 'success'

class CheckCenter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered', 'not_centered'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ExchangeCube')
        if True: #check for center. LIDAR?
            return 'centered'
        else:
            return 'not_centered'

#unfinished
class TurnForCube(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered', 'not_centered'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ExchangeCube')
        if True: #check for center. LIDAR?
            return 'centered'
        else:
            return 'not_centered'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

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
