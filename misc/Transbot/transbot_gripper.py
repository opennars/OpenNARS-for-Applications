from time import sleep
from transbot_msgs.srv import *
from transbot_msgs.msg import *
from geometry_msgs.msg import Twist
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
    jointangle(7, s_angle)
def arm_servo8(s_angle):
    jointangle(8, s_angle)
def arm_servo9(s_angle):
    jointangle(9, s_angle)

def arm_down():
    arm_servo7(180)
    sleep(0.5)
    arm_servo7(180)
    sleep(0.5)
    arm_servo8(220)
    sleep(0.5)
    arm_servo8(220)
    sleep(0.5)
    arm_servo7(55)
    sleep(0.5)
    arm_servo7(55)
    sleep(0.5)

def arm_up():
    arm_servo7(180)
    sleep(0.5)
    arm_servo7(180)
    sleep(0.5)
    arm_servo8(30)
    sleep(0.5)
    arm_servo8(30)
    sleep(0.5)
    arm_servo7(210)
    sleep(0.5)
    arm_servo7(210)
    sleep(0.5)

def init_pose():
    arm_up()
    arm_servo9(30)
    sleep(0.5)
    arm_servo9(30)
    sleep(0.5)

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
                joints[joint.id] = joint.angle
    except Exception:
        print("//couldn't get joint angle")
    return joints

def read_gripper_angle():
    ang = {}
    while 9 not in ang:
        ang = angles()
    return ang[9]

def close_gripper():
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
            return True
        if comparable(target_angle, current_angle):
            target_angle += step_size
            arm_servo9(target_angle)
            sleep(0.7)
        else:
            return False
    
def open_gripper():
    arm_servo9(30)
    sleep(1)

#buy two additional degrees of freedom by moving base

pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
angular = 0.5
linear = 0.3

def left():
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = angular
    pub_vel.publish(twist)
    sleep(0.2)
    twist.linear.x = 0
    twist.angular.z = 0
    pub_vel.publish(twist)

def right():
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = -angular
    pub_vel.publish(twist)
    sleep(0.2)
    twist.linear.x = 0
    twist.angular.z = 0
    pub_vel.publish(twist)

def forward():
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = 0
    pub_vel.publish(twist)
    sleep(0.2)
    twist.linear.x = 0
    twist.angular.z = 0
    pub_vel.publish(twist)

def backward():
    twist = Twist()
    twist.linear.x = -linear
    twist.angular.z = 0
    pub_vel.publish(twist)
    sleep(0.2)
    twist.linear.x = 0
    twist.angular.z = 0
    pub_vel.publish(twist)

picked = False
def getPicked():
    return picked

def pick():
    global picked
    if picked:
        return
    arm_down()
    forward()
    feedback = close_gripper()
    if not feedback:
        open_gripper()
    picked = feedback
    arm_up()
    backward()
    return feedback

def drop():
    global picked
    if not picked:
        return
    forward()
    arm_down()
    open_gripper()
    arm_up()
    backward()
    backward()
    backward()
    picked = False

print("//transbot_gripper.py go!")
