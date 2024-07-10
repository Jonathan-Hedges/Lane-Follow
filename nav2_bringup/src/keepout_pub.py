import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import os
from PIL import Image
import numpy as np
import math

class MapServer(Node):
    def __init__(self):
        super().__init__('map_server')
        #self.declare_parameter('map_yaml_file', 'path/to/map.yaml')
        self.map_yaml_file = self.get_parameter('yaml_filename').get_parameter_value().string_value
        self.zone_yaml_files = self.get_parameter('yaml_filenames').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.map_msg = None  # This will store the map data loaded from the YAML file

        self.zone_boundaries_dict = {
            "zone1": [3, 368],
            "zone2": [202, 6],
            "zone3": [5, 680],
            "zone4": [564, 2],
            "zone5": [566, 629]
        }

        self.get_map_service = self.create_service(GetMap, 'get_map', self.get_map_callback)
        self.load_map_service = self.create_service(LoadMap, 'load_map', self.load_map_callback)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # Subscribe to the robot's pose topic
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',  # Change this topic name if your robot's pose is published elsewhere
            self.pose_callback,
            10)  # Adjust the QoS as needed

        self.current_pose = None  # To store the robot's current pose


        # Make name prefix for services
        service_prefix = self.get_name() + "/"

        # Create a service that provides the occupancy grid
        self.occ_service = self.create_service(
            GetMap, service_prefix + self.service_name, self.get_map_callback)

        # Create a publisher using the QoS settings to emulate a ROS1 latched topic
        self.occ_pub = self.create_publisher(
            OccupancyGrid, self.topic_name,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))

        # Create a service that loads the occupancy grid from a file
        self.load_map_service = self.create_service(
            LoadMap, service_prefix + self.load_map_service_name, self.load_map_callback)
        
        # Only try to load map if parameter was set
        if self.map_yaml_file:
            # Initialize response for LoadMap service
            response = LoadMap.Response()

            try:
                if self.load_map_from_yaml(self.map_yaml_file, response):
                    self.occ_pub.publish(self.map_msg)
                else:
                    self.get_logger().error(f"Failed to load map yaml file: {self.map_yaml_file}")
            except Exception as e:
                self.get_logger().error(f"Exception occurred: {str(e)}")


    def pose_callback(self, msg):
        """Callback function for the robot's pose."""
        self.current_pose = msg.pose.pose
        # self.get_logger().info(f"Current Pose: {self.current_pose}")


    def get_map_callback(self, request, response):
        if self.get_current_state().label != 'active':
            self.get_logger().warn("Received GetMap request but not in ACTIVE state, ignoring!")
            return response

        self.get_logger().info("Handling GetMap request")
        response.map = self.map_msg
        return response

    def load_map_callback(self, request, response):
        if self.get_current_state().label != 'active':
            self.get_logger().warn("Received LoadMap request but not in ACTIVE state, ignoring!")
            response.result = LoadMap.Response.RESULT_UNDEFINED_FAILURE
            return response

        self.get_logger().info("Handling LoadMap request")
        
        if self.load_map_response_from_yaml(request.map_url, response):
            self.map_publisher.publish(self.map_msg)  # Publish the new map
        return response

    def load_map_response_from_yaml(self, yaml_file, response):
        if not os.path.exists(yaml_file):
            response.result = LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST
            return False

        try:
            with open(yaml_file, 'r') as file:
                map_data = yaml.safe_load(file)
                # Assume map_data contains necessary fields. You'll need to convert this data into an OccupancyGrid message.
                # This is a placeholder for the conversion logic.
                self.map_msg = OccupancyGrid()  # Populate this with actual map data
                # Update the map_msg header, frame_id, etc., as needed.
                self.map_msg.header.frame_id = "map"
                response.map = self.map_msg
                response.result = LoadMap.Response.RESULT_SUCCESS
                return True
        except Exception as e:
            self.get_logger().error(f"Failed to load map from YAML: {e}")
            response.result = LoadMap.Response.RESULT_INVALID_MAP_METADATA
            return False
    
    def load_map_from_file(self, image_file_name, resolution, origin, negate, occupied_thresh, free_thresh, mode):
        try:
            img = Image.open(image_file_name).convert('RGBA')
            width, height = img.size

            # Initialize OccupancyGrid message
            msg = OccupancyGrid()
            msg.info.width = width
            msg.info.height = height
            msg.info.resolution = resolution
            msg.info.origin.position.x = origin[0]
            msg.info.origin.position.y = origin[1]
            msg.info.origin.position.z = 0.0
            # Set orientation here if needed

            # Convert image to numpy array
            img_data = np.array(img)
            # Process image data to occupancy values
            # This is a simplified version; adjust processing as needed
            occupancy_data = self.process_image_data(img_data, negate, occupied_thresh, free_thresh, mode)

            msg.data = occupancy_data.flatten().tolist()
            self.map_msg = msg
        except Exception as e:
            self.get_logger().error(f'Failed to load map from file: {e}')

    def process_image_data(self, img_data, negate, occupied_thresh, free_thresh, mode):
        # Implement image processing to convert to occupancy values
        # Placeholder implementation
        return np.zeros((img_data.shape[0], img_data.shape[1]), dtype=np.int8)

    def load_map_from_yaml(self, yaml_file):
        try:
            with open(yaml_file, 'r') as file:
                yaml_data = yaml.safe_load(file)
                # Extract necessary fields from yaml_data and call load_map_from_file
                # Placeholder for extracting and using data
        except Exception as e:
            self.get_logger().error(f'Failed to load map from YAML: {e}')



def main(args=None):
    rclpy.init(args=args)
    map_server = MapServer()
    rclpy.spin(map_server)
    map_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()