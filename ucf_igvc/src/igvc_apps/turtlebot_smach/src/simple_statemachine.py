#!/usr/bin/env python
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseResult
from move_base_msgs.msg import MoveBaseActionResult
from smach_ros import ServiceState
from smach_ros import SimpleActionState
from std_srvs.srv import Empty
from waypoint_manager_msgs.srv import NextWaypoint
from waypoint_manager_msgs.srv import PushWaypoint
from geodesy import utm
from roboteq_msgs.msg import Status

global vehicle_state

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.wait_for_service('autonomous_mode')
        rospy.loginfo("Entering autonomous mode")
        rospy.wait_for_service('advance_waypoint')
        rospy.wait_for_service('backtrack_waypoint')
        rospy.wait_for_service('next_waypoint')
        rospy.wait_for_service('push_new_waypoint')
        return 'succeeded'


class GetNextWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['waypoint_out'])

    def execute(self, userdata):
        try:
            get_waypoint_proxy = rospy.ServiceProxy('next_waypoint', NextWaypoint)
            waypoint_response = get_waypoint_proxy()
            userdata.waypoint_out = waypoint_response.waypoint
        except rospy.ServiceException:
            return 'aborted'

        return 'succeeded'


class MovingToWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['waypoint_in'])

    def execute(self, userdata):
        move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server(rospy.Duration(5))
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'utm'
        goal_pose.target_pose.header.stamp = rospy.Time.now()

        # First point specified is latitude, second is longitude
        utm_waypoint = utm.fromLatLong(userdata.waypoint_in.position.x, userdata.waypoint_in.position.y, 250)
        goal_pose.target_pose.pose.position.x = utm_waypoint.easting
        goal_pose.target_pose.pose.position.y = utm_waypoint.northing
        goal_pose.target_pose.pose.position.z = 246
        goal_pose.target_pose.pose.orientation.x = 0
        goal_pose.target_pose.pose.orientation.y = 0
        goal_pose.target_pose.pose.orientation.z = 0
        goal_pose.target_pose.pose.orientation.w = 1

        rospy.loginfo(goal_pose)
        
        move_base_client.send_goal(goal_pose)
        rospy.loginfo("Sent goal")
        move_base_client.wait_for_result()
        result = move_base_client.get_state()
        if (result == GoalStatus.SUCCEEDED):
            return 'succeeded'
        else:
            return 'failed'


class AdvanceWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        advance_waypoint_proxy = rospy.ServiceProxy('advance_waypoint', Empty)
        advance_waypoint_proxy()
        return 'succeeded'


class BacktrackWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        backtrack_waypoint_proxy = rospy.ServiceProxy('backtrack_waypoint', Empty)
        backtrack_waypoint_proxy()
        return 'succeeded'

def serviceHandler():
    return

def flagCallback(data):
    vehicle_state = data.autonomous_state
    if vehicle_state:
        try:
            # Start a ROS service to signal we have received true for autonomous mode
            # Really really goofy but no spinonce in rospy
            server = rospy.Service("autonomous_mode", Empty, serviceHandler)
        except rospy.ServiceException, e:
            return

def main():
    rospy.init_node('test_state_machine')

    rospy.loginfo("Starting subscriber")
    autonomous_sub = rospy.Subscriber("/roboteq_driver/status", Status, flagCallback)

    # Create the state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open top level state machine
    with sm:
        # Add intial idle state.
        smach.StateMachine.add("Idle", Idle(), transitions={'succeeded':'GetNextWaypoint'})

        # Add GetNextWaypoint state.
        smach.StateMachine.add("GetNextWaypoint", GetNextWaypoint(),
                               transitions={'succeeded':'MovingToWaypoint',
                                            'aborted':'finish'},
                               remapping={'waypoint_out':'waypoint_in'})

        # Add MovingToWaypoint state.
        smach.StateMachine.add("MovingToWaypoint", MovingToWaypoint(),
                               transitions={'succeeded':'AdvanceWaypoint',
                                            'failed':'finish'})

        # Add AdvanceWaypoint state.
        smach.StateMachine.add("AdvanceWaypoint", AdvanceWaypoint(),
                               transitions={'succeeded':'GetNextWaypoint'})

    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    main()
