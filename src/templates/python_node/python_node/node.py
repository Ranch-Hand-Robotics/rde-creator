#!/usr/bin/env python3
"""
{{package_name}} - {{package_description}}

This ROS 2 node provides {{node_name}} functionality with the following features:
{{#if include_publisher}}- Message publishing on topic: {{topic_name}}
{{/if}}{{#if include_subscriber}}- Message subscription from topic: {{topic_name}}
{{/if}}{{#if include_service}}- Service server for: {{service_name}}
{{/if}}{{#if include_client}}- Service client for: {{service_name}}
{{/if}}{{#if include_timer}}- Timer-based periodic tasks ({{timer_period}}s interval)
{{/if}}{{#if include_parameters}}- Dynamic parameter handling
{{/if}}
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
{{#if include_publisher}}
from std_msgs.msg import String
{{/if}}{{#if include_subscriber}}
from std_msgs.msg import String
{{/if}}{{#if include_service}}
from std_srvs.srv import Trigger
{{/if}}{{#if include_client}}
from std_srvs.srv import Trigger
{{/if}}
import logging


class {{node_name|pascalcase}}Node(Node):
    """
    ROS 2 Node implementation for {{package_name}}.

    This node demonstrates best practices for ROS 2 Python development including:
    - Proper node lifecycle management
    - Quality of Service (QoS) configuration
    - Error handling and logging
    - Parameter management
    - Timer-based execution
    """

    def __init__(self):
        super().__init__('{{node_name}}')

        # Configure logging
        self.logger = self.get_logger()
        self.logger.info('Initializing {{node_name}} node')

        # QoS profile for reliable communication
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        {{#if include_publisher}}
        # Publisher setup
        self.publisher_ = self.create_publisher(
            String,
            '{{topic_name}}',
            self.qos_profile
        )
        self.logger.info(f'Publisher created for topic: {{topic_name}}')
        {{/if}}

        {{#if include_subscriber}}
        # Subscriber setup
        self.subscription = self.create_subscription(
            String,
            '{{topic_name}}',
            self.listener_callback,
            self.qos_profile
        )
        self.subscription  # prevent unused variable warning
        self.logger.info(f'Subscriber created for topic: {{topic_name}}')
        {{/if}}

        {{#if include_service}}
        # Service server setup
        self.service = self.create_service(
            Trigger,
            '{{service_name}}',
            self.service_callback
        )
        self.logger.info(f'Service server created for: {{service_name}}')
        {{/if}}

        {{#if include_client}}
        # Service client setup
        self.client = self.create_client(Trigger, '{{service_name}}')
        self.logger.info(f'Service client created for: {{service_name}}')
        {{/if}}

        {{#if include_timer}}
        # Timer setup
        self.timer = self.create_timer({{timer_period}}, self.timer_callback)
        self.logger.info(f'Timer created with period: {{timer_period}} seconds')
        {{/if}}

        {{#if include_parameters}}
        # Parameter setup
        self.declare_parameter('example_param', 'default_value')
        self.logger.info('Parameters declared')
        {{/if}}

        self.logger.info('{{node_name}} node initialization complete')

    {{#if include_subscriber}}
    def listener_callback(self, msg):
        """
        Callback function for subscriber.

        Args:
            msg: Received message
        """
        self.logger.info(f'Received message: {msg.data}')

        # Process the received message
        # Add your message processing logic here
        processed_data = f"Processed: {msg.data}"

        {{#if include_publisher}}
        # Optionally publish processed data
        response_msg = String()
        response_msg.data = processed_data
        self.publisher_.publish(response_msg)
        self.logger.debug(f'Published processed message: {processed_data}')
        {{/if}}
    {{/if}}

    {{#if include_service}}
    def service_callback(self, request, response):
        """
        Callback function for service requests.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response with result
        """
        self.logger.info('Service request received')

        # Process the service request
        # Add your service logic here
        response.success = True
        response.message = "Service executed successfully"

        self.logger.info(f'Service response: {response.message}')
        return response
    {{/if}}

    {{#if include_client}}
    def call_service(self):
        """
        Call the service asynchronously.
        """
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.error('Service not available')
            return

        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """
        Callback for service response.

        Args:
            future: Future object containing the service response
        """
        try:
            response = future.result()
            self.logger.info(f'Service response: {response.message}')
        except Exception as e:
            self.logger.error(f'Service call failed: {e}')
    {{/if}}

    {{#if include_timer}}
    def timer_callback(self):
        """
        Timer callback function executed periodically.
        """
        self.logger.debug('Timer callback executed')

        {{#if include_publisher}}
        # Publish a message on timer
        msg = String()
        msg.data = f'Hello from {{node_name}} at {self.get_clock().now().to_msg().sec}'
        self.publisher_.publish(msg)
        self.logger.debug(f'Published: {msg.data}')
        {{/if}}

        {{#if include_parameters}}
        # Check parameter changes
        param_value = self.get_parameter('example_param').value
        self.logger.debug(f'Parameter value: {param_value}')
        {{/if}}

        {{#if include_client}}
        # Optionally call service on timer
        if self.get_clock().now().nanoseconds % 10 == 0:  # Every 10 seconds
            self.call_service()
        {{/if}}
    {{/if}}

    {{#if include_parameters}}
    def parameter_callback(self, param):
        """
        Callback for parameter changes.

        Args:
            param: Parameter that changed
        """
        self.logger.info(f'Parameter {param.name} changed to: {param.value}')
    {{/if}}


def main(args=None):
    """
    Main function to run the ROS 2 node.

    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)

    try:
        node = {{node_name|pascalcase}}Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(f'Node execution failed: {e}')
    finally:
        # Clean shutdown
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()