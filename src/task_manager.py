#!/usr/bin/env python3
import signal
import sys
import os
from dotenv import load_dotenv

import rospy
import smach

from state_machines.request_handler import RequestInterpreterSM
from state_machines.move.move_one import MoveOneSM
from state_machines.control.control_module import CtrlModuleSM

def signal_handler(signum, frame):
    rospy.signal_shutdown('Shutting down on Ctrl+C')
    sys.exit(1)

signal.signal(signal.SIGINT, signal_handler)

class TaskManager:
    def __init__(self):

        load_dotenv()
        
        self.robot_name = os.getenv('ROBOT_NAME') # ex) tb3_0

        rospy.loginfo(">> Robot name is..." + self.robot_name)
        
        rospy.init_node("task_manager_node_" + self.robot_name)

        self.sm = smach.StateMachine(outcomes=['exit', 'done'])

        self.sm.userdata.robot_name = self.robot_name
        self.sm.userdata.action = "" # ex) MOVE / MODULE
        self.sm.userdata.command = "" # ex) MOVE tb3_0 seconds x z delay ...[todo]
        self.sm.userdata.data = "" # ex) x 0 0 0 0 z delay [todo]

        with self.sm:
            smach.StateMachine.add('REQUEST', RequestInterpreterSM(robot_name = self.robot_name),
                                   transitions={'MOVE':'MOVE',
                                                'MODULE':'MODULE'})
            smach.StateMachine.add('MOVE', MoveOneSM(),
                                   transitions={'arrive': 'REQUEST'})
            smach.StateMachine.add('MODULE', CtrlModuleSM(),
                                   transitions={'complete': 'REQUEST'})

if __name__ == "__main__":

    task_manager= TaskManager()
    outcome = task_manager.sm.execute()

    rospy.loginfo("[TaskManager] final state is %s. done.", outcome)
    
    rospy.spin()
