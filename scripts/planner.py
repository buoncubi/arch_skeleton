#! /usr/bin/env python

import random
import rospy
# Import constant name defined to structure the architecture.
from arch_skeleton import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from arch_skeleton.msg import Point, PlanFeedback, PlanResult
from arch_skeleton.srv import GetPose
import arch_skeleton  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER


# An action server to simulate motion planning.
# Given a target position, it retrieve the current robot position from the 
# `robot-state` node, and return a plan as a set of via points.
class PlaningAction(object):

    def __init__(self):
        # Get random-based parameters used by this server
        self._random_plan_points = rospy.get_param(anm.PARAM_PLANNER_POINTS, [2, 8])
        self._random_plan_time = rospy.get_param(anm.PARAM_PLANNER_TIME, [0.1, 1])
        self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      arch_skeleton.msg.PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = (f'`{anm.ACTION_PLANNER}` Action Server initialised. It will create random path with a number of point '
                   f'spanning in [{self._random_plan_points[0]}, {self._random_plan_points[1]}). Each point will be generated '
                   f'with a delay spanning in [{self._random_plan_time[0]}, {self._random_plan_time[1]}).')
      
    # The callback invoked when a client set a goal to the `planner` server.
    # This function will return a list of random points (i.e., the plan) when the fist point
    # is the current robot position (retrieved from the `robot-state` node), while the last 
    # point is the `goal` position (given as input parameter). The plan will contain 
    # a random number of other points, which spans in the range 
    # [`self._random_plan_points[0]`, `self._random_plan_points[1]`). To simulate computation,
    # each point is added to the plan with a random delay spanning in the range 
    # [`self._random_plan_time[0]`, `self._random_plan_time[1]`).
    def execute_callback(self, goal):
        # Get the input parameters to compute the plan, i.e., the start (or current) and target positions.
        start_point = _get_pose_client()
        target_point = goal.target

        # Check if the start and target positions are correct. If not, this service will be aborted.
        if start_point is None or target_point is None:
            log_msg = 'Cannot have `None` start point nor target_point. This service will be aborted!.'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        if not(self._is_valid(start_point) and self._is_valid(target_point)):
            log_msg = (f'Start point ({start_point.x}, {start_point.y}) or target point ({target_point.x}, '
                       f'{target_point.y}) point out of the environment. This service will be aborted!.')
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        
        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []
        feedback.via_points.append(start_point)
        # Publish the feedback and wait to simulate computation.
        self._as.publish_feedback(feedback)
        delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
        rospy.sleep(delay)

        # Get a random number of via points to be included in the plan.
        number_of_points = random.randint(self._random_plan_points[0], self._random_plan_points[1] + 1)
        log_msg = f'Server is planning {number_of_points + 1} points...'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Generate the points of the plan.
        for i in range(1, number_of_points):
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Server has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()  
                return
            # Generate a new random point of the plan.
            new_point = Point()
            new_point.x = random.uniform(0, self._environment_size[0])
            new_point.y = random.uniform(0, self._environment_size[1])
            feedback.via_points.append(new_point)
            if i < number_of_points - 1:
                # Publish the new random point as feedback to the client.
                self._as.publish_feedback(feedback)
                # Wait to simulate computation.
                delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
                rospy.sleep(delay)
            else:
                # Append the target point to the plan as the last point.
                feedback.via_points.append(target_point)

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points
        self._as.set_succeeded(result)
        log_msg = 'Motion plan succeeded with plan: '
        log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + '), ' for point in result.via_points)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # Check if the point is within the environment bounds, i.e.
    # x: [0, `self._environment_size[0]`], and y: [0, `self._environment_size[1]`].
    def _is_valid(self, point):
        return 0.0 <= point.x <= self._environment_size[0] and 0.0 <= point.y <= self._environment_size[1]


# Retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node.
def _get_pose_client():
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        # Call the service and get a response with the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        # Log service response.
        log_msg = f'Retrieving current robot position from the `{anm.NODE_ROBOT_STATE}` node as: ({pose.x}, {pose.y}).'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return pose
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.    
    rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()
