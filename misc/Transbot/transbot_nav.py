import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID

rospy.init_node('NARTest')

#pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
pub_cancelgoal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()

def OpGo(x, y, z=0, w=1, frame_id = 'map'):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id # "base_link";
    goal.target_pose.header.stamp = rospy.Time.now()
    # The location of the target point
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # The posture of the target point. z=sin(angle/2) w=cos(angle/2)
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()

def OpStop():
    pub_cancelgoal.publish(GoalID())

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

