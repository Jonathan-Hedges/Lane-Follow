#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import os
from rclpy.duration import Duration
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityFactory
from ament_index_python.packages import get_package_share_directory


"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    #     # Create a client for the SpawnEntity service
    # node = rclpy.create_node('spawn_entity_client')
    # client = node.create_client(SpawnEntity, 'spawn_entity')

    # # Wait for the service to be available
    # while not client.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('SpawnEntity service not available, waiting again...')

    # # Create a request with the desired parameters
    # request = SpawnEntity.Request()
    # request.name = 'cube'  # Name of the entity to spawn
    # request.allow_renaming = False
    # request.sdf = ''  # SDF description as a string, if applicable

    # bringup_dir = get_package_share_directory('nav2_bringup')
    # default_value=os.path.join(bringup_dir, 'worlds', 'cube.sdf')

    # request.sdf_filename = default_value  # Path to SDF file, if applicable
    # request.clone_name = ''  # Name of entity to clone, if applicable
    # request.pose = Pose()  # Set the desired pose
    # request.pose.position.x = 1.0
    # request.pose.position.y = 1.0
    # request.pose.orientation.w = 1.0
    # request.relative_to = 'world'

    # # Call the service and wait for the response
    # future = client.call_async(request)
    # rclpy.spin_until_future_complete(node, future)
    # try:
    #     response = future.result()
    #     node.get_logger().info('SpawnEntity call succeeded')
    # except Exception as e:
    #     node.get_logger().error('Service call failed %r' % (e,))

    # # Don't forget to shutdown
    # rclpy.shutdown()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Zone locations mapped as locations in the map [top left x,y , bottom right x,y]
    zones = [
        [-0.5, -14, 4, -16.5],
        [9, 20, 12, 16],
        [-0.5, 2, 3, -1],
        [27.5, 20, 32, 16],
        [27.5, -10, 32, -14.5]
    ]
    # Go to our demos first goal pose
    # Go to top left zone
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 11.0
    goal_pose.pose.position.y = 18.0
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.orientation.z = 0.0

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    # why do none of these change map functions work?
    # navigator.changeMap('/nav2_ws/src/navigation2/nav2_bringup/maps/zone_free.yaml')
    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(
                        feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation

            # Bug!! seconds = seconds/100 for some reason
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180000.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=1800.0):
                goal_pose.pose.position.x = 0.0
                goal_pose.pose.position.y = 0.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # By allowing this, you must call the launch file again
    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
