import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from dao.RobotDao import updateRobotModuleState, selectModuleStateByRobotID, resetRobotStatus # mySQL
from dao.moduleDao import selectDegreeByState, selectStateByDegree # mySQL

DISPLAY_TIME = 10.0

class ControlRequest(smach.State):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["done", "none"],
                                    input_keys=['data'], 
                                    output_keys=['data'])

        self.ctrl_pub = rospy.Publisher('task_manager/ctrl_module_req', String, queue_size=1)

    def execute(self, user_data):
        goal_data = user_data.data

        self.ctrl_pub.publish(goal_data)
        rospy.loginfo("[CtrlModule] ctrl_module_req is published now: %s", goal_data)

        return 'done'
        
class InControl(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, 'task_manager/ctrl_module_res', String, self.update_status,
                                        input_keys=['robot_name', 'data'], 
                                        output_keys=['data'])
        
    def update_status(self, user_data, res_msg):
        result = str(res_msg.data).split()

        robot_name = user_data.robot_name

        newState:float = 0

        # update current module state of robot
        try:
            (moduleState,) = selectModuleStateByRobotID(robotID=robot_name) # 현재 state 불러오기

            cur_degZ, cur_degX = selectDegreeByState(state= moduleState) # 현재 state기준 degree 불러오기

            # 목표 degree = 각도 변위값 + 현재 degree
            goal_degZ = float(result[0]) + cur_degZ
            goal_degX = float(result[1]) + cur_degX
                
            # 업데이트할 state = 목표 degre로 state불러오기
            (newState,) = selectStateByDegree(degZ=goal_degZ, degX=goal_degX)
                
            # 새로운 state로 업데이트
            updateRobotModuleState(robotID=robot_name, moduleState=float(newState))
            rospy.logwarn(f"[CtrlModule] Robot {robot_name}'s module state has now been updated to {newState}.")
        except:
            rospy.logerr("[CtrlModule] Failed to update the robot's module state.")

        # 로봇 상태 IDLE로 초기화
        resetRobotStatus(robotID=robot_name)
        rospy.logwarn(f"[MoveOne] robot {robot_name}'s status has been initialized.")
        
        return False


class CtrlModuleSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["complete"],
                                    input_keys=['robot_name', 'data'], 
                                    output_keys=['data'])
        with self:
            self.add('CONTROL_REQUEST', ControlRequest(),
                     transitions={'done': 'IN_CONTROL',
                                  'none': 'complete'})
            self.add('IN_CONTROL', InControl(),
                     transitions={'invalid': 'complete',
                                'valid': 'IN_CONTROL',
                                'preempted':'IN_CONTROL'})
