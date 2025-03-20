from action import Action
from typing import List

class ParallelAction(Action):
    def __init__(self, action_list:List[Action]):
        self.__action_list:List[Action] = action_list

# TODO: Nest printing of actions one after the other through some way
        for a in self.__action_list[:]:
            if a is None:
                #NodeHandle.node_handle.get_logger().error("Invalid action added to list")
                print("invalid added to list")
                self.__action_list.remove(a)
        
    def start(self):
        for a in self.__action_list:
            a.start()

    def update(self):
        for a in self.__action_list:
            a.update()

    def done(self):
        pass

    def isFinished(self) -> bool:
        retval = True
        for a in self.__action_list[:]:
            if not a.isFinished():
                retval &= False
            else:
                a.done()
                self.__action_list.remove(a)

        return retval

    def preempt(self):
        for a in self.__action_list:
            a.preempt()
    
    def __str__(self) -> str:
        return f"Parallel: {', '.join(str(a) for a in self.__action_list)}"
