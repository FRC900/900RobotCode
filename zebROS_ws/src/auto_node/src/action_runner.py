#!/usr/bin/env python3
from action import Action
from series_action import SeriesAction
from parallel_action import ParallelAction
from wait_action import WaitAction
from typing import List
import rospy

class ActionRunner:
    def __init__(self) -> None:
        self.__active_action_list:List[Action] = []

    def preempt_all_actions(self):
        rospy.logerr_throttle(5, "Preempting all actions since disabled!")
        for a in self.__active_action_list:
            a.preempt()

    def reset_action_list(self):
        self.__active_action_list:List[Action] = []

    # does not actually do anything if affected systems overlap
    def start_action(self, new_action:Action):
        try:
            #TODO test timing and maybe come up with a more efficient solution
            self.__active_action_list[:] = list(filter(lambda a: not any(s in a.affectedSystems() for s in new_action.affectedSystems()), self.__active_action_list))
            self.__active_action_list.append(new_action)
            new_action.start() 
        except:
            rospy.loginfo("Expection encounted starting action")
        
            #NodeHandle.node_handle.get_logger().error("Exception encountered starting action")

    def loop(self, disabled : bool):
        num_actions = len(self.__active_action_list)
        if disabled != True:
            if num_actions > 0:
                ############DEBUG
                for a in self.__active_action_list:
                    #NodeHandle.node_handle.get_logger().info(f"Active Action: {str(a)}", throttle_duration_sec=1)
                    rospy.loginfo_throttle(1, f"Active action: {str(a)}")
                #################

                if any(a.isFinished() for a in self.__active_action_list):
                    for a in self.__active_action_list:
                        if a.isFinished():
                            ############DEBUG
                            #NodeHandle.node_handle.get_logger().info(f"Finished Action: {str(a)}", throttle_duration_sec=1)
                            rospy.loginfo_throttle(1, f"Finished Action: {str(a)}")
                            #################
                            a.done()

                    self.__active_action_list[:] = list(filter(lambda a: not a.isFinished(), self.__active_action_list))

                for a in self.__active_action_list:
                    a.update()
        else:
            if num_actions > 0:
                self.preempt_all_actions()
                self.__active_action_list = []

if __name__ == "__main__":
    rospy.init_node("action_runner")
    runner = ActionRunner()
    to_run = SeriesAction([
                ParallelAction([ 
                    SeriesAction([WaitAction(1), WaitAction(1)]),
                    SeriesAction([WaitAction(3), WaitAction(3)])
                                ])
                            ])
    runner.start_action(to_run)
    print("Here")
    r = rospy.Rate(100)
    while True:
        try:
            runner.loop(disabled=False)
            r.sleep()
        except KeyboardInterrupt:
            exit(0)