import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from dao.RobotDao import updateRobotVelocity, updateRobotStatus # mySQL

IDLE = "IDLE"

class MoveRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"],
                                    input_keys=['data'], 
                                    output_keys=['data'])

        self.move_pub = rospy.Publisher('task_manager/move_req', String, queue_size=1)

    def execute(self, user_data):
        goal_data = user_data.data
        
        self.move_pub.publish(goal_data)
        rospy.loginfo("[MoveOne] move_req is published now: %s", goal_data)

        return 'done'

class OnTheMove(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, 'task_manager/move_res', String, self.update_status,
                                        input_keys=['robot_name', 'data'], 
                                        output_keys=['data'])
        
    def update_status(self, user_data, res_msg):
        result = str(res_msg.data).split()

        robot_name = user_data.robot_name

        # 새로운 cmd_vel 데이터로 업데이트
        updateRobotVelocity(robotID=robot_name, seconds=result[0], linX=result[1], angZ=result[2])
        rospy.logwarn(f"[MoveOne] Robot {robot_name}'s cmd_vel has now been updated to [{result[0]}, {result[1]}, {result[2]}].")

        rospy.loginfo(f"[MoveOne] Robot {robot_name} arrived.")

        # 로봇 상태 IDLE로 초기화
        updateRobotStatus(robotID=robot_name, status=IDLE)
        rospy.logwarn(f"[MoveOne] robot {robot_name}'s status updated to {IDLE}.")

        return False
       

class MoveOneSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["arrive"],
                                    input_keys=['robot_name', 'data'], 
                                    output_keys=['data'])
        
        with self:
            self.add('MOVE_REQUESET', MoveRequest(),
                     transitions={'done': 'ON_THE_MOVE'})
            self.add('ON_THE_MOVE', OnTheMove(),
                     transitions={'invalid': 'arrive',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})
