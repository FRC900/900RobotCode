from action import Action
from typing import List

class EarlyExitParallelAction(Action):
    """
    Runs all actions (`wait_for_action_list` + `also_run_action_list`) in parallel, and exits + preempts all once all actions in `wait_for_action_list` are done.
    """
    def __init__(self, wait_for_action_list: List[Action], also_run_action_list: List[Action]):
        self.__wait_for_action_list: List[Action] = wait_for_action_list
        self.__also_run_action_list: List[Action] = also_run_action_list

# TODO: Nest printing of actions one after the other through some way
        for a in self.__wait_for_action_list[:]:
            if a is None:
                print("invalid added to list")
                self.__wait_for_action_list.remove(a)
        
        for a in self.__also_run_action_list[:]:
            if a is None:
                print("invalid added to list")
                self.__also_run_action_list.remove(a)
        
    def start(self):
        for a in self.__wait_for_action_list:
            a.start()
        for a in self.__also_run_action_list:
            a.start()

    def update(self):
        for a in self.__wait_for_action_list:
            a.update()
        for a in self.__also_run_action_list:
            a.update()

    def done(self):
        for a in self.__also_run_action_list:
            a.preempt()
        pass

    def isFinished(self) -> bool:
        retval = True
        for a in self.__wait_for_action_list[:]:
            if not a.isFinished():
                retval &= False
            else:
                a.done()
                self.__wait_for_action_list.remove(a)
        
        for a in self.__also_run_action_list[:]:
            if a.isFinished():
                a.done()
                self.__also_run_action_list.remove(a)

        return retval

    def preempt(self):
        for a in self.__wait_for_action_list:
            a.preempt()
        for a in self.__also_run_action_list:
            a.preempt()
    
    def __str__(self) -> str:
        return f"Eatly Exit Parallel: waiting on {', '.join(str(a) for a in self.__wait_for_action_list)} and also running {', '.join(str(a) for a in self.__also_run_action_list)}"
