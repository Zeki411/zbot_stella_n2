import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, PoseStamped
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener
from transforms3d.quaternions import quat2mat, mat2quat

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


from .wavefront_frontier_detection import WavefrontFrontierDetection, OccupancyGrid2d
import time


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('wavefront_frontier_explorer')
        self.get_logger().info('WavefrontFrontierExplorer initialized')

        # # Declare parameter to use simulation time
        # self.declare_parameter('use_sim_time', True)

        self.frontier_marker_pub = self.create_publisher(
            Marker, '/frontier_markers', QoSProfile(depth=1)
        )

        # Subscribe to the /map topic
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.slam_map_callback, 
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        # self.odom_sub = self.create_subscription(
        #     Odometry, '/odom', self.odom_callback,
        #     QoSProfile(
        #         durability=QoSDurabilityPolicy.VOLATILE,
        #         reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #         history=QoSHistoryPolicy.KEEP_LAST,
        #         depth=1
        #     )
        # )

        self._nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tfcheck_timer = self.create_timer(0.05, self.tfcheck_callback)

        self.frontier_detector = WavefrontFrontierDetection()

        self._process_next_frontier = True

        # Allow some time for all nodes to start up and broadcast their frames
        time.sleep(5)

    def __send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self._nav2_action_client.wait_for_server()
        self._send_goal_future = self._nav2_action_client.send_goal_async(goal_msg, feedback_callback=self.__feedback_callback)
        self._send_goal_future.add_done_callback(self.__goal_response_callback)

    def __goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__get_result_callback)

    def __feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def __get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal result: {result}')
        
        status = future.result().status
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal failed with status: ABORTED')
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal was canceled')
        else:
            self.get_logger().warn('Goal failed with status: {0}'.format(status))
        
        self._process_next_frontier = True


    def tfcheck_callback(self):
        try:
            # # Get the current time from the clock (which will be sim time if available)
            # current_time = self.get_clock().now()
            
            # Get the transformation from 'map' to 'odom'
            transform_map_to_odom = self.tf_buffer.lookup_transform('map', 'odom', 
                                                                    self.tf_buffer.get_latest_common_time('map', 'odom'))

            # Get the transformation from 'odom' to 'base_footprint'
            transform_odom_to_base = self.tf_buffer.lookup_transform('odom', 'base_footprint', 
                                                                     self.tf_buffer.get_latest_common_time('odom', 'base_footprint'))

            # self.get_logger().info(f'Got transform from map to odom: {transform_map_to_odom}')

            # # Extract the translation and rotation from 'map' to 'odom'
            trans_map_to_odom = np.array([transform_map_to_odom.transform.translation.x,
                                          transform_map_to_odom.transform.translation.y,
                                          transform_map_to_odom.transform.translation.z])

            rot_map_to_odom = np.array([transform_map_to_odom.transform.rotation.w,
                                        transform_map_to_odom.transform.rotation.x,
                                        transform_map_to_odom.transform.rotation.y,
                                        transform_map_to_odom.transform.rotation.z])
            rot_mat_map_to_odom = quat2mat(rot_map_to_odom)

            # Extract the translation and rotation from 'odom' to 'base_footprint'
            trans_odom_to_base = np.array([transform_odom_to_base.transform.translation.x,
                                           transform_odom_to_base.transform.translation.y,
                                           transform_odom_to_base.transform.translation.z])
            rot_odom_to_base = np.array([transform_odom_to_base.transform.rotation.w,
                                         transform_odom_to_base.transform.rotation.x,
                                         transform_odom_to_base.transform.rotation.y,
                                         transform_odom_to_base.transform.rotation.z])
            rot_mat_odom_to_base = quat2mat(rot_odom_to_base)

            # self.get_logger().info(f'trans map to odom {trans_map_to_odom}')
            # self.get_logger().info(f'rot map to odom {rot_mat_map_to_odom}')
            # self.get_logger().info(f'trans odom to base {trans_odom_to_base}')
            # self.get_logger().info(f'rot odom to base {rot_mat_odom_to_base}')

            # Combine the transformations
            trans_map_to_base = trans_map_to_odom + rot_mat_map_to_odom @ trans_odom_to_base
            rot_mat_map_to_base = rot_mat_map_to_odom @ rot_mat_odom_to_base

            # Convert back to quaternion
            rot_map_to_base = mat2quat(rot_mat_map_to_base)

            # Create the Pose message
            pose = Pose()
            pose.position.x = trans_map_to_base[0]
            pose.position.y = trans_map_to_base[1]
            pose.position.z = trans_map_to_base[2]
            pose.orientation.x = rot_map_to_base[1]
            pose.orientation.y = rot_map_to_base[2]
            pose.orientation.z = rot_map_to_base[3]
            pose.orientation.w = rot_map_to_base[0]

            # Update the pose in your frontier detector or other relevant component
            self.frontier_detector.update_pose(pose)
            
            # Optionally, log the pose for debugging
            self.get_logger().info(f"Robot position: {pose.position.x}, {pose.position.y}, {pose.position.z}")
            self.get_logger().info(f"Robot orientation: {pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w}")
            
        except Exception as e:
            # Log a message if the transformation lookup fails
            self.get_logger().info('Failed to get transform from map to base_footprint: {}'.format(e))


    def odom_callback(self, msg):
        # mark for odom
        self.get_logger().info(f'Updating pose to {msg}')
        odom_pose = Pose()
        odom_pose.position = msg.pose.pose.position
        odom_pose.orientation = msg.pose.pose.orientation
        self.frontier_detector.update_pose(odom_pose)

    def slam_map_callback(self, msg):

        if not self._process_next_frontier:
            return
        
        self.frontier_detector.update_map(OccupancyGrid2d(msg))
        frontiers = self.frontier_detector.get_frontiers()
        self.get_logger().info(f'Found {len(frontiers)} frontiers.')

        # Create and publish marker for all frontiers
        frontier_marker = Marker()
        frontier_marker.header.frame_id = 'map'
        frontier_marker.header.stamp = self.get_clock().now().to_msg()
        frontier_marker.ns = 'frontiers'
        frontier_marker.id = 0
        frontier_marker.type = Marker.POINTS
        frontier_marker.action = Marker.ADD
        frontier_marker.pose.orientation.w = 1.0
        frontier_marker.scale.x = 0.05
        frontier_marker.scale.y = 0.05
        frontier_marker.color.r = 1.0
        frontier_marker.color.a = 1.0

        for frontier in frontiers:
            point = Point()
            point.x = float(frontier[0])
            point.y = float(frontier[1])
            point.z = 0.0  # Ensure z is set
            frontier_marker.points.append(point)

        self.frontier_marker_pub.publish(frontier_marker)

        # Find the closest frontier
        frontier_distance = []
        closest_frontier = None
        for frontier in frontiers:
            frontier = np.array(frontier)
            robot_position = np.array([self.frontier_detector.current_pose.position.x, self.frontier_detector.current_pose.position.y])
            frontier_distance.append(np.linalg.norm(frontier - robot_position))

        if frontiers:
            closest_frontier = frontiers[np.argmin(frontier_distance)]
            self.get_logger().info(f'Closest frontier is at {closest_frontier}')

            # Create and publish marker for the closest frontier
            closest_frontier_marker = Marker()
            closest_frontier_marker.header.frame_id = 'map'
            closest_frontier_marker.header.stamp = self.get_clock().now().to_msg()
            closest_frontier_marker.ns = 'closest_frontier'
            closest_frontier_marker.id = 1
            closest_frontier_marker.type = Marker.SPHERE
            closest_frontier_marker.action = Marker.ADD
            closest_frontier_marker.pose.orientation.w = 1.0
            closest_frontier_marker.scale.x = 0.2
            closest_frontier_marker.scale.y = 0.2
            closest_frontier_marker.scale.z = 0.2
            closest_frontier_marker.color.g = 1.0
            closest_frontier_marker.color.r = 0.0
            closest_frontier_marker.color.b = 0.0
            closest_frontier_marker.color.a = 1.0

            point = Point()
            point.x = float(closest_frontier[0])
            point.y = float(closest_frontier[1])
            point.z = 0.0  # Ensure z is set
            closest_frontier_marker.pose.position = point
            self.frontier_marker_pub.publish(closest_frontier_marker)
        
        current_pose_marker = Marker()
        current_pose_marker.header.frame_id = 'map'
        current_pose_marker.header.stamp = self.get_clock().now().to_msg()
        current_pose_marker.id = 2
        current_pose_marker.type = Marker.SPHERE
        current_pose_marker.action = Marker.ADD
        current_pose_marker.pose.orientation.w = 1.0
        current_pose_marker.scale.x = 0.2
        current_pose_marker.scale.y = 0.2
        current_pose_marker.scale.z = 0.2
        current_pose_marker.color.b = 1.0
        current_pose_marker.color.r = 0.0
        current_pose_marker.color.g = 0.0
        current_pose_marker.color.a = 1.0
        current_pose_point = Point()
        current_pose_point.x = self.frontier_detector.current_pose.position.x
        current_pose_point.y = self.frontier_detector.current_pose.position.y
        current_pose_point.z = 0.0
        current_pose_marker.pose.position = current_pose_point
        self.frontier_marker_pub.publish(current_pose_marker)

        if closest_frontier is not None:
            # Send the robot to the closest frontier
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = closest_frontier[0]
            goal_pose.pose.position.y = closest_frontier[1]
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            

            self.__send_goal(goal_pose)
            self._process_next_frontier = False
        
    
        
        
        




    

def main():
    rclpy.init()
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
