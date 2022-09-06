from time import sleep
from transbot_msgs.srv import *
from transbot_msgs.msg import *
import rospy

runtime = 500  #1000
RobotArm_client = rospy.ServiceProxy("/CurrentAngle", RobotArm)
pub_Arm = rospy.Publisher("/TargetAngle", Arm, queue_size=1)
rospy.init_node('NARTest')
joints = {7: Joint(), 8: Joint(), 9: Joint()}
joints[7].id = 7
joints[7].angle = 210
joints[7].run_time = runtime
joints[8].id = 8
joints[8].angle = 30
joints[8].run_time = runtime
joints[9].id = 9
joints[9].angle = 30
joints[9].run_time = runtime


def jointangle(id, angle):
    global joints
    arm = Arm()
    joints[id].angle = angle
    sleep(float(runtime)/1000.0)
    arm.joint.append(joints[7])
    arm.joint.append(joints[8])
    arm.joint.append(joints[9])
    pub_Arm.publish(arm)

# 控制机械臂关节运动
# Control the joint movement of the manipulator
def arm_servo7(s_angle):
    #if s_angle > 210 or s_angle < 160:
    #    return
    jointangle(7, s_angle)
    
# 控制机械臂关节运动
# Control the joint movement of the manipulator
def arm_servo8(s_angle):
    #if s_angle > 180 or s_angle < 30:
    #    return
    jointangle(8, s_angle)
    
# 控制机械臂关节运动
# Control the joint movement of the manipulator
def arm_servo9(s_angle):
    #if s_angle > 180 or s_angle < 30:
    #    return
    jointangle(9, s_angle)

    
def init_pose():
    arm_servo7(210)
    arm_servo8(30)
    arm_servo9(30)
#init_pose()

init_pose()

def angles():
    RobotArm_client.wait_for_service()
    request = RobotArmRequest()
    request.apply = "getJoint"
    joints = {}
    try:
        response = RobotArm_client.call(request)
        if isinstance(response, RobotArmResponse):
            print(response.RobotArm.joint)
            for joint in response.RobotArm.joint:
                #print(str(joint))
                #joints.append(joint.angle)
                joints[joint.id] = joint.angle
    except Exception:
        print("//couldn't get joint angle")
    #print("Arm_joints: ", joints)
    return joints

def read_gripper_angle():
    ang = {}
    while 9 not in ang:
        ang = angles()
    return ang[9]

print(angles())


def close_gripper():
    #ret_changes = []
    target_angle = 30
    step_size = 5.0
    tolerance = 14.0
    comparable = lambda x,y: abs(x-y) <= tolerance
    last_angles = []
    last_target_angles = []
    while(target_angle <= 180-step_size):
        current_angle = read_gripper_angle()
        print("current angle and target angle %d %d" % (current_angle, target_angle))
        last_angles.append(current_angle)
        last_target_angles.append(target_angle)
        if current_angle >= 80 and len(last_angles) >= 2 and last_angles[-2] != last_angles[-1] and last_angles[-2] + 3 >= last_angles[-1]:
            arm_servo9(last_target_angles[-2])
            print("FEEDBACK STOP 1")
            sleep(0.7)
            break
        if comparable(target_angle, current_angle):
            target_angle += step_size
            arm_servo9(target_angle)
            sleep(0.7)
        else:
            break
    #return ret_changes
    return []
    
def open_gripper():
    arm_servo9(30)
    sleep(1)

def pick():
    #pick:
    arm_servo7(180)
    sleep(1)

    arm_servo8(220)
    sleep(1)

    arm_servo7(55)
    sleep(1)
    arm_servo8(220)
    sleep(1)

    close_gripper()

    arm_servo7(180)
    sleep(1)
    arm_servo8(30)
    sleep(1)

    arm_servo7(210)
    sleep(1)
    
def drop():
    #drop:
    arm_servo7(180)
    sleep(1)

    arm_servo8(220)
    sleep(1)

    arm_servo7(55)
    sleep(1)
    arm_servo8(220)
    sleep(1)

    open_gripper()

    arm_servo7(180)
    sleep(1)
    arm_servo8(30)
    sleep(1)

    arm_servo7(210)
    sleep(1)

#pick()
#drop()
