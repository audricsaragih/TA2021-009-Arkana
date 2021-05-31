import rospy
import actionlib
from std_msgs.msg import String, Int64
from test_tutorials.msg import Pwm
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def callbackPose(data):
    msgfix.header = data.header
    msgfix.pose = data.pose.pose

def callbackStop(data):
    #goalpub.publish(msgfix)
    client.cancel_goal()
    statuspub.publish(2)  

def callbackMap(data):
    mappub = rospy.Publisher('map_continues', OccupancyGrid, queue_size=10)
    rateMap = rospy.Rate(6)
    while not rospy.is_shutdown():
        mappub.publish(data)
        #rospy.loginfo('publish map')
        rateMap.sleep()

def active_cb():
    rospy.loginfo("active: goal is being processed")
    statuspub.publish(1)

def feedback_cb(feedback):
    rospy.loginfo('feedback')

def done_cb(status, result):
    if status == 2:
        rospy.loginfo("Goal received cancelation")
    elif status == 3:
        rospy.loginfo("Goal reached")
    elif status == 4:
        rospy.loginfo("Goal was aborted by action server")
    elif status == 5:
        rospy.loginfo("Goal has been rejected by action server")
    elif status == 8:
        rospy.loginfo("Goal received a cancel request before start executing")
    else :
        rospy.loginfo("Unknown reason")
        statuspub.publish(4)

def callbackGoal(data):
    goal.target_pose = data
    client.send_goal(goal, done_cb, active_cb, feedback_cb)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        statuspub.publish(5)
    else:
        result = client.get_result()
        if result:
            rospy.loginfo("Goal execution done!")
            statuspub.publish(3)

def listener():
    #rospy.init_node('interface_node', anonymous=True)
    msgfix.header.stamp = rospy.Time.now()
    msgfix.header.frame_id = 'map'
    msgfix.pose.position.x = 0.0
    msgfix.pose.position.y = 0.0
    msgfix.pose.position.z = 0.0
    msgfix.pose.orientation.x = 0.0
    msgfix.pose.orientation.y = 0.0
    msgfix.pose.orientation.z = 0.0
    msgfix.pose.orientation.w = 0.0

    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callbackPose)
    rospy.Subscriber("stop_goal_new", Twist, callbackStop)
    rospy.Subscriber("map", OccupancyGrid, callbackMap)
    rospy.Subscriber("goal_sent", PoseStamped, callbackGoal)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        posepub.publish(msgfix)
        #rospy.loginfo(msgfix)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('interface_node', anonymous=True)

        statuspub = rospy.Publisher('robot_status', Int64, queue_size=10)
        posepub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        msgfix = PoseStamped()

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        goal = MoveBaseGoal()       

        listener()
    except rospy.ROSInterruptException:
        pass
