#! /usr/bin/env python3
#ros package
from geometry_msgs.msg import PoseStamped,Pose,PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

#rclpy libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import math

#other libraries
#from command_pose.srv import Nav2GoPose
from amr_service.srv import NavInitPose,NavToPose ,NavCancleTask ,NavThroughPose

class Nav2_commander(Node):
    def __init__(self):
        super().__init__('nav2_commander')
        self.navigator = BasicNavigator()
        #timer_period = 0.01
        #self.timer = self.create_timer(timer_period,self.timer_callback)
        self.service_ini = self.create_service(NavInitPose, 'nav2_init_pose', self.nav2_init_pose)
        self.service_to_pose = self.create_service(NavToPose, 'nav2_to_pose', self.nav2_to_pose)
        self.service_through_pose = self.create_service(NavThroughPose, 'nav2_through_pose', self.nav2_through_pose)
        self.service_cancle_task = self.create_service(NavCancleTask, 'nav2_cancle_task', self.nav2_cancle_task)
        
        
        self.get_logger().info("Service server is ready to receive requests.")

    def nav2_cancle_task(self,request, response):
        self.get_logger().info(f'Incoming CANCLE request:{request.target_pose}')
        #self.get_logger().info(f'getResult:{self.navigator.nav_to_pose_client._get_result}')
        
        self.navigator.cancelTask()
        self.navigator.waitUntilNav2Active()
        self.navigator.clearAllCostmaps()
        response.complete = True
        return response
    
    def nav2_init_pose(self,request, response):
        self.navigator.cancelTask()
        self.get_logger().info(f'Incoming init request:{request.target_pose}')
        command_target = Pose()
        command_target = request.target_pose
        self.navigator.waitUntilNav2Active()

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = command_target.orientation.z
        q.w = command_target.orientation.w
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if norm == 0:
            raise ValueError("Invalid input: Quaternion has zero length")
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = command_target.position.x
        initial_pose.pose.position.y = command_target.position.y
        initial_pose.pose.orientation.z = q.z
        initial_pose.pose.orientation.w = q.w

        #self.get_logger().info(f'Incoming type request:{type(command_target.position.x)}')

        # initial_pose.pose.position.x = 3.451
        # initial_pose.pose.position.y = 2.15
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 0.999
        #initial_pose.pose.covariance = [0.1] * 36
        self.navigator.setInitialPose(initial_pose)
        
        response.complete = True
        return response
    def nav2_through_pose(self,request, response):
        self.navigator.cancelTask()
        self.get_logger().info(f'Incoming to request: {request.target_pose}')
        flag = True
        command_target = Pose()
        command_target = request.target_pose 
        self.navigator.waitUntilNav2Active()

        # You may use the navigator to clear or obtain costmaps
        self.navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()
        # Go to our demos first goal pose
        goal_poses = []
        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = 'map'
        goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()
        # goal_pose1.pose.position.x = -13.0
        # goal_pose1.pose.position.y = 17.57
        goal_pose1.pose.position.x = 18.0
        goal_pose1.pose.position.y = 3.57

        goal_pose1.pose.orientation.w = 1.0
        goal_pose1.pose.orientation.z = 0.0
        goal_poses.append(goal_pose1)

        # additional goals can be appended
        goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
        # goal_pose2.pose.position.x = -11.0
        # goal_pose2.pose.position.y = 17.57
        goal_pose1.pose.position.x = 18.0
        goal_pose1.pose.position.y = 4.57

        goal_pose2.pose.orientation.w = 1.0
        goal_pose2.pose.orientation.z = 0.0
        goal_poses.append(goal_pose2)

        goal_pose3 = PoseStamped()
        goal_pose3.header.frame_id = 'map'
        goal_pose3.header.stamp = self.navigator.get_clock().now().to_msg()
        # goal_pose3.pose.position.x = -9.0
        # goal_pose3.pose.position.y = 17.57
        goal_pose1.pose.position.x = 18.0
        goal_pose1.pose.position.y = 5.57
        goal_pose3.pose.orientation.w = 1.0
        goal_pose3.pose.orientation.z = 0.0
        goal_poses.append(goal_pose3)

    # sanity check a valid path exists
    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

        self.navigator.goThroughPoses(goal_poses)
        response.complete = True
        #print(f"Sent reponse: {response.complete}")
        
        return response

    def nav2_to_pose(self,request, response):
        self.navigator.cancelTask()
        self.get_logger().info(f'Incoming to request: {request.target_pose}')
        flag = True
        command_target = Pose()
        command_target = request.target_pose 
        self.navigator.waitUntilNav2Active()

        # You may use the navigator to clear or obtain costmaps
        self.navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()
        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = command_target.position.x
        goal_pose.pose.position.y = command_target.position.y
        goal_pose.pose.orientation.z = command_target.orientation.z
        goal_pose.pose.orientation.w = command_target.orientation.w

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)
        self.navigator.goToPose(goal_pose)
        # i = 0
        # while not self.navigator.isTaskComplete():
        #     rclpy.spin_once(self)
        #     i = i + 1
        #     if i % 5 == 0:
        #         print('Wait task complete ...')

        # result = self.navigator.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     print('Goal succeeded!')
        # elif result == TaskResult.CANCELED:
        #     print('Goal was canceled!')
        # elif result == TaskResult.FAILED:
        #     print('Goal failed!')
        # else:
        #     print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

        response.complete = True
        #print(f"Sent reponse: {response.complete}")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Nav2_commander()
    try:
        while rclpy.ok(): # while the node isn't shut down
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node has stopped cleanly.')
    except SystemExit:
        node.get_logger().info('Node is complete.')
    except BaseException as exc:
        type = exc.__class__.__name__
        node.get_logger().error(f'{type} exception in node has occured.')
        raise # raise without argument = raise the last exception
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

