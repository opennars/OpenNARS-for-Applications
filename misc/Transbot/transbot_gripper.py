from time import sleep
from transbot_msgs.srv import *
from transbot_msgs.msg import *
from geometry_msgs.msg import Twist
import rospy

hastrailer = False
def get_hastrailer():
    return hastrailer
def set_hastrailer(has):
    global hastrailer
    hastrailer = has
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

# 控制机械臂关节运动
# Control the joint movement of the manipulator
def jointangle(id, angle):
    global joints
    arm = Arm()
    joints[id].angle = angle
    sleep(float(runtime)/1000.0)
    arm.joint.append(joints[7])
    arm.joint.append(joints[8])
    arm.joint.append(joints[9])
    pub_Arm.publish(arm)

def arm_down():
    jointangle(7, 180)
    sleep(0.5)
    jointangle(7, 180)
    sleep(0.5)
    jointangle(8, 220)
    sleep(0.5)
    jointangle(8, 220)
    sleep(0.5)
    jointangle(7, 55)
    sleep(0.5)
    jointangle(7, 55)
    sleep(0.5)

def arm_up():
    jointangle(7, 180)
    sleep(0.5)
    jointangle(7, 180)
    sleep(0.5)
    jointangle(8, 60 if hastrailer else 30)
    sleep(0.5)
    jointangle(8, 60 if hastrailer else 30)
    sleep(0.5)
    jointangle(7, 210)
    sleep(0.5)
    jointangle(7, 210)
    sleep(0.5)

def init_pose():
    arm_up()
    jointangle(9, 30)
    sleep(0.5)
    jointangle(9, 30)
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

def close_gripper(target_angle = 30):
    step_size = 5.0
    tolerance = 14.0
    comparable = lambda x,y: abs(x-y) <= tolerance
    last_angles = []
    last_target_angles = []
    while(target_angle <= 180-step_size):
        if target_angle >= 160:
            print("//FEEDBACK STOP 0")
            return False, target_angle
        current_angle = read_gripper_angle()
        print("//current angle and target angle %d %d" % (current_angle, target_angle))
        last_angles.append(current_angle)
        last_target_angles.append(target_angle)
        if current_angle >= 80 and len(last_angles) >= 2 and last_angles[-2] != last_angles[-1] and last_angles[-2] + 3 >= last_angles[-1]:
            jointangle(9, last_target_angles[-2])
            print("//FEEDBACK STOP 1")
            sleep(0.7)
            return True, target_angle
        if comparable(target_angle, current_angle):
            target_angle += step_size
            jointangle(9, target_angle)
            sleep(0.7)
        else:
            return False, target_angle
    return False, target_angle
    
def open_gripper():
    jointangle(9, 30)
    sleep(1)
    jointangle(9, 30)
    sleep(1)

#buy two additional degrees of freedom by moving base

pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
angular = 0.4
linear = 0.3

def left(angular=angular):
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = angular
    pub_vel.publish(twist)
    sleep(0.2)
    twist.linear.x = 0
    twist.angular.z = 0
    pub_vel.publish(twist)

def right(angular=angular):
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = -angular
    pub_vel.publish(twist)
    sleep(0.2)
    twist.linear.x = 0
    twist.angular.z = 0
    pub_vel.publish(twist)

def forward(linear=linear):
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = 0
    pub_vel.publish(twist)
    sleep(0.2)
    twist.linear.x = 0
    twist.angular.z = 0
    pub_vel.publish(twist)

def backward(linear=linear):
    twist = Twist()
    twist.linear.x = -linear
    twist.angular.z = 0
    pub_vel.publish(twist)
    sleep(0.2)
    twist.linear.x = 0
    twist.angular.z = 0
    pub_vel.publish(twist)

picked = False
def setPicked(value):
    global picked
    picked = value
def getPicked():
    return picked

def pick(force=False):
    global picked
    if picked and not force:
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

def drop(force=False):
    global picked
    if not picked and not force:
        return
    forward()
    arm_down()
    open_gripper()
    arm_up()
    backward()
    backward()
    backward()
    backward()
    right()
    right()
    picked = False

def drop_trailer(force=False, t=0.7):
    global picked
    if not picked and not force:
        return
    for i in range(32):
        right(angular=0.6)
        sleep(t)
    forward(linear=0.6)
    sleep(t)
    forward(linear=0.6)
    sleep(t)
    open_gripper()
    backward(linear=0.6)
    sleep(t)
    backward(linear=0.6)
    sleep(t)
    for i in range(30):
        left(angular=0.6)
        sleep(t)
    picked = False

print("//transbot_gripper.py go!")
