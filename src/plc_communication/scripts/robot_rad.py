#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.client import Client
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Float32MultiArray

class CostmapParamChangerNode(Node):
    def __init__(self):
        super().__init__('costmap_param_changer')
        self.get_logger().info("Node initialized.")
        
        # Create clients for the /local_costmap and /global_costmap nodes' parameters
        self.local_param_client = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        self.global_param_client = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')

        # Wait until the /local_costmap and /global_costmap nodes are available
        while not self.local_param_client.wait_for_service(timeout_sec=1.0) or not self.global_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /local_costmap or /global_costmap service...")

        # Subscribe to the state_param topic
        self.create_subscription(Float32MultiArray, 'param_state', self.param_state_callback, 10)

        # To track the current parameter values
        self.last_robot_radius = None
        self.last_inflation_radius = None
        self.last_width = None

        # To track pending requests
        self.pending_requests = []

    def param_state_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warn("Invalid data received. Expecting three values: robot_radius, inflation_radius, width.")
            return
        
        # Extract values from the received message
        new_robot_radius = msg.data[0]/10
        new_inflation_radius = msg.data[1]/10
        new_width = int(msg.data[2])

        #self.get_logger().info(f"Received state_param: robot_radius={new_robot_radius}, inflation_radius={new_inflation_radius}, width={new_width}")

        # Set parameters only if they have changed
        self.change_costmap_parameters(new_robot_radius, new_inflation_radius, new_width)

    def change_costmap_parameters(self, new_robot_radius, new_inflation_radius, new_width):
        # Set parameters for /local_costmap one by one asynchronously, only if values have changed
        if self.last_robot_radius != new_robot_radius:
            self.set_local_costmap_param('robot_radius', Parameter.Type.DOUBLE, new_robot_radius)
            #self.last_robot_radius = new_robot_radius  # Update the stored value

        if self.last_inflation_radius != new_inflation_radius:
            self.set_local_costmap_param('inflation_layer.inflation_radius', Parameter.Type.DOUBLE, new_inflation_radius)
            #self.last_inflation_radius = new_inflation_radius  # Update the stored value

        if self.last_width != new_width:
            self.set_local_costmap_param('width', Parameter.Type.INTEGER, new_width)
            self.set_local_costmap_param('height', Parameter.Type.INTEGER, new_width)
            self.last_width = new_width  # Update the stored value

        # Set parameters for /global_costmap (only robot_radius in this case)
        if self.last_robot_radius != new_robot_radius:
            self.set_global_costmap_param('robot_radius', Parameter.Type.DOUBLE, new_robot_radius)
            self.last_robot_radius = new_robot_radius  # Update the stored value
        
        if self.last_inflation_radius != new_inflation_radius:
            self.set_global_costmap_param('inflation_layer.inflation_radius', Parameter.Type.DOUBLE, new_inflation_radius)
            self.last_inflation_radius = new_inflation_radius  # Update the stored value

    def set_local_costmap_param(self, param_name, param_type, param_value):
        param = Parameter(param_name, param_type, param_value)
        request = SetParameters.Request()
        request.parameters = [param.to_parameter_msg()]

        self.get_logger().info(f"Setting parameter '{param_name}' for /local_costmap to {param_value}")

        future = self.local_param_client.call_async(request)
        self.pending_requests.append(future)  # Track pending requests

    def set_global_costmap_param(self, param_name, param_type, param_value):
        param = Parameter(param_name, param_type, param_value)
        request = SetParameters.Request()
        request.parameters = [param.to_parameter_msg()]

        self.get_logger().info(f"Setting parameter '{param_name}' for /global_costmap to {param_value}")

        future = self.global_param_client.call_async(request)
        self.pending_requests.append(future)  # Track pending requests

    def handle_pending_requests(self):
        # Check the status of all pending requests
        for future in self.pending_requests[:]:
            if future.done():
                # Remove completed requests from the list
                self.pending_requests.remove(future)
                if future.result() is not None:
                    self.get_logger().info("Successfully set parameter.")
                else:
                    self.get_logger().error("Failed to set parameter.")

def main(args=None):
    rclpy.init(args=args)
    node = CostmapParamChangerNode()

    try:
        while rclpy.ok():
            node.handle_pending_requests()  # Check if any requests are completed
            rclpy.spin_once(node)  # Allow subscriptions and service calls to proceed
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
