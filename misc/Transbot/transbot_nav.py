import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
pub_cancelgoal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

def OpGo(x, y, z=0, w=1, frame_id = 'map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id # "base_link";
    pose.header.stamp = rospy.Time.now()
    # The location of the target point
    pose.pose.position.x = x
    pose.pose.position.y = y
    # The posture of the target point. z=sin(angle/2) w=cos(angle/2)
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    pub_goal.publish(pose)

def OpStop():
    pub_cancelgoal.publish(GoalID())

rospy.init_node('NARTest')

#Code version of "rosrun tf tf_echo /map base_link | grep Translation:"
from threading import Thread, Lock
import subprocess
import time
lock = Lock()
translation = [0,0,0]
rotation = [0, 0, 0, 0]

def getLocation():
    lock.acquire()
    copyTrans = [x for x in translation]
    copyRot = [x for x in rotation]
    lock.release()
    return (copyTrans, copyRot)

def updateLocation():
    global translation, rotation
    proc = subprocess.Popen(['rosrun', 'tf', 'tf_echo', '/map', 'base_link'],stdout=subprocess.PIPE)
    while True:
      line = str(proc.stdout.readline())
      if "Translation:" in line:
          lock.acquire()
          translation = [float(x.replace(",","")) for x in line.split("[")[1].split("]")[0].split(" ")]
          lock.release()
      if "Rotation:" in line:
          lock.acquire()
          rotation = [float(x.replace(",","")) for x in line.split("[")[1].split("]")[0].split(" ")]
          lock.release()
      time.sleep(0.1)
      
Thread(target=updateLocation).start()

print("//transbot_nav.py go!")

