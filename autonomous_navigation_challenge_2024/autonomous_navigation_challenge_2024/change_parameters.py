import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, Parameter
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import ListParameters
import time

class ChangeParameters(Node):
    def __init__(self, node_name="", parameter_name="", parameter_value=None):
        super().__init__("change_parameters")
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Initializing change_parameters node")

        self.declare_parameter("node_name", node_name,)
        self.declare_parameter("parameter_name", parameter_name,)
        self.declare_parameter("parameter_value", parameter_value)

        node_name = self.get_parameter("node_name").get_parameter_value().string_value
        parameter_name = self.get_parameter("parameter_name").get_parameter_value().string_value
        parameter_value = self.get_parameter("parameter_value").get_parameter_value()

        service_name = node_name + "/set_parameters"

        self._waitForNodeToActivate(node_name)

        cli=self.create_client(SetParameters, service_name)
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} service not available, waiting again...')

        # Create a Parameter
        # param = self.create_parameter(parameter_name, self.create_parameterValue(parameter_type, parameter_value))
        param = self.create_parameter(parameter_name, parameter_value)

        # Create a SetParameters.Request and add the parameters
        self.req = SetParameters.Request()

        # Add the parameter to the request
        self.req.parameters.append(param)

        # Send the request
        self.future = cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.done() and self.future.result() is not None:
            if all(result.successful for result in self.future.result().results):
                self.get_logger().info(f'Parameters {parameter_name} successfully set')
            else:
                self.get_logger().info('Failed to set parameters')
        else:
            self.get_logger().info('Service call failed')
        # rclpy.shutdown()

    def create_parameter(self, name: str, value: ParameterValue) -> Parameter:
        param = Parameter()
        param.name = name
        param.value = value
        return param
    
    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.get_logger().info(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/list_parameters'
        state_client = self.create_client(ListParameters, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{node_service} service not available, waiting...')

        req = ListParameters.Request()
        names = []
        while len(names) == 0:
            self.get_logger().info(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                names = future.result().result.names
                self.get_logger().info("The node is active")
            time.sleep(2)
        return
    
def main():
    rclpy.init()
    executor = SingleThreadedExecutor()

    node = ChangeParameters()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
