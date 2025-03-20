from action import Action
from typing import List

class SeriesAction(Action):
    def __init__(self, action_list:List[Action]):
        self.__current_action_index:int = -1
        self.__action_list:List[Action] = action_list

        for a in self.__action_list[:]:
            if a is None:
                #NodeHandle.node_handle.get_logger().error("Invalid action added to list")
                print("invalid action added to list")
                self.__action_list.remove(a)

        self.__current_action:Action = None
        pass

    def start(self):
        print("start called for series")
        self.__current_action = None
        self.__current_action_index = 0

    def update(self):
        if self.__current_action == None:
            if self.__current_action_index >= len(self.__action_list):
                return

            self.__current_action = self.__action_list[self.__current_action_index]
            self.__current_action_index += 1
            self.__current_action.start()

        self.__current_action.update()

        if self.__current_action.isFinished():
            self.__current_action.done()
            self.__current_action = None

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__current_action == None and self.__current_action_index == len(self.__action_list)

    def preempt(self):
        for a in self.__action_list:
            a.preempt()
    
    def __str__(self) -> str:
        return f"Series: {str(self.__current_action)}"
