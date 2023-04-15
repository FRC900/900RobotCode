import rospy

from frc_msgs.msg import ButtonBoxState2023
from behavior_actions.msg import AlignAndPlaceGrid2023Goal

def callback(msg: ButtonBoxState2023):
    if msg.gridSelectConeLeftButton:
        print(f"Cone left button pressed at {msg.header.stamp.to_sec()}")
    if msg.gridSelectConeRightButton:
        print(f"Cone right button pressed at {msg.header.stamp.to_sec()}")
    if msg.gridSelectCubeButton:
        print(f"Cube button pressed at {msg.header.stamp.to_sec()}")

def main():

    rospy.init_node('debug_auto_place')
    rospy.Subscriber("/frcrobot_rio/button_box_states", ButtonBoxState2023, callback)
    rospy.Subscriber("/align_and_place_grid/goal", AlignAndPlaceGrid2023Goal, lambda msg: print(f"Action called at {msg.header.stamp.to_sec()}"))

    rospy.spin()

if __name__ == "__main__":
    main()