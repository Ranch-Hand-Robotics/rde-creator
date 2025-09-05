#!/usr/bin/env python3
"""
Test suite for {{package_name}}.

This module contains unit tests for the {{node_name}} node functionality.
"""

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
{{#if include_service}}
from std_srvs.srv import Trigger
{{/if}}
import time


class Test{{node_name|pascalcase}}Node(unittest.TestCase):
    """
    Test cases for {{node_name}} node.
    """

    @classmethod
    def setUpClass(cls):
        """Set up test class."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Clean up test class."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        self.node = Node('test_node')

    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()

    {{#if include_publisher}}
    def test_publisher_creation(self):
        """Test that publisher is created correctly."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        self.assertIsNotNone(test_node.publisher_)
        test_node.destroy_node()
    {{/if}}

    {{#if include_subscriber}}
    def test_subscriber_creation(self):
        """Test that subscriber is created correctly."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        self.assertIsNotNone(test_node.subscription)
        test_node.destroy_node()
    {{/if}}

    {{#if include_service}}
    def test_service_creation(self):
        """Test that service is created correctly."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        self.assertIsNotNone(test_node.service)
        test_node.destroy_node()
    {{/if}}

    {{#if include_timer}}
    def test_timer_creation(self):
        """Test that timer is created correctly."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        self.assertIsNotNone(test_node.timer)
        test_node.destroy_node()
    {{/if}}

    {{#if include_parameters}}
    def test_parameter_declaration(self):
        """Test that parameters are declared correctly."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        param_value = test_node.get_parameter('example_param').value
        self.assertEqual(param_value, 'default_value')
        test_node.destroy_node()
    {{/if}}

    {{#if include_publisher}}
    def test_message_publishing(self):
        """Test message publishing functionality."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()

        # Create a message subscriber to verify publishing
        received_messages = []

        def message_callback(msg):
            received_messages.append(msg.data)

        subscriber = self.node.create_subscription(
            String,
            '{{topic_name}}',
            message_callback,
            10
        )

        # Wait for subscription to be ready
        time.sleep(0.1)

        # Publish a test message
        test_message = String()
        test_message.data = "test_message"
        test_node.publisher_.publish(test_message)

        # Wait for message to be received
        timeout = 5.0
        start_time = time.time()
        while len(received_messages) == 0 and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertEqual(len(received_messages), 1)
        self.assertEqual(received_messages[0], "test_message")

        test_node.destroy_node()
        self.node.destroy_subscription(subscriber)
    {{/if}}

    {{#if include_service}}
    def test_service_callback(self):
        """Test service callback functionality."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()

        # Create a service client
        client = self.node.create_client(Trigger, '{{service_name}}')

        # Wait for service to be available
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        # Call the service
        request = Trigger.Request()
        future = client.call_async(request)

        # Wait for response
        timeout = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        self.assertTrue(future.done())
        response = future.result()
        self.assertTrue(response.success)
        self.assertEqual(response.message, "Service executed successfully")

        test_node.destroy_node()
        self.node.destroy_client(client)
    {{/if}}


if __name__ == '__main__':
    unittest.main()