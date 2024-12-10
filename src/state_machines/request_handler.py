import rospy
import smach
import smach.state
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

"""
@request    MOVE    {robot_id} {data} {data}...
@request    MODULE  {robot_id} {data} {data}...
"""
class RequestMonitor(smach_ros.MonitorState):
    def __init__(self, robot_name):
        smach_ros.MonitorState.__init__(self, '/task_scheduler/' + robot_name, String, self.check_command,
                                        input_keys=['command'],
                                        output_keys=['action', 'command'])
        
    def check_command(self, user_data, res_msg):
        user_data.command = str(res_msg.data)

        rospy.loginfo("[TaskManager] I got new command. (%s)", user_data.command)
        
        return False
    
class Request2State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MOVE', 'MODULE'],
                             io_keys=['action', 'command', 'data'])
        
    def execute(self, user_data):
        
        cmd_list = str(user_data.command).split()

        user_data.action = cmd_list[0] 
        user_data.data = ' '.join(map(str, cmd_list[2:]))
        
        rospy.loginfo("[TaskManager] I saved state \'%s\' in TaskManager", user_data.action)
        
        return user_data.action

class RequestInterpreterSM(smach.StateMachine):
    def __init__(self, robot_name):
        smach.StateMachine.__init__(self, outcomes=['MOVE','MODULE'],
                                    input_keys=['action', 'command', 'data'],
                                    output_keys=['action', 'command', 'data'])
        
        with self:
            self.add('REQ_MONITOR', RequestMonitor(robot_name=robot_name),
                     transitions={'invalid': 'REQ_INTERPRET',
                                  'valid': 'REQ_MONITOR',
                                  'preempted':'REQ_MONITOR'})
            self.add('REQ_INTERPRET', Request2State())
            