import rospy
import smach
import smach.state
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from dao.RobotDao import updateRobotStatus

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
    def __init__(self, robot_name):
        smach.State.__init__(self, outcomes=['MOVE', 'MODULE'],
                             io_keys=['action', 'command', 'data'])
        
        self.robot_name = robot_name
        self.robot_status_pub = rospy.Publisher('/task_scheduler/robot_status', String, queue_size=1) # -> robot monitor
        
    def execute(self, user_data):
        
        cmd_list = str(user_data.command).split()

        user_data.action = cmd_list[0] 
        user_data.data = ' '.join(map(str, cmd_list[2:]))
        
        # 로봇 상태 업데이트 (MOVE, MODULE)
        updateRobotStatus(robotID=self.robot_name, status=user_data.action)
        self.robot_status_pub.publish(f"{self.robot_name} {user_data.action}")
        rospy.loginfo("[TaskManager] Robot stauts is updated to \'%s\'", user_data.action)
        
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
            self.add('REQ_INTERPRET', Request2State(robot_name=robot_name))