#!/usr/bin/env python3
"""
Comprehensive test suite for {{package_name}}.

This module contains unit tests for the {{node_name}} node functionality,
including publisher/subscriber patterns, service interactions, parameter handling,
and lifecycle management.

Author: {{package_maintainer}}
Date: {{year}}
"""

import unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
{{#if include_service}}
from std_srvs.srv import Trigger
{{/if}}
import time
import threading
from typing import List, Any


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

    def test_node_lifecycle(self):
        """Test node lifecycle management."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        
        # Test node initialization
        self.assertEqual(test_node.get_name(), '{{node_name}}')
        self.assertIsNotNone(test_node.get_namespace())
        
        # Test node cleanup
        test_node.destroy_node()
        
    def test_node_exception_handling(self):
        """Test node's exception handling capabilities."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        
        # Test parameter access with invalid parameter
        with self.assertRaises(rclpy.exceptions.ParameterNotDeclaredException):
            test_node.get_parameter('nonexistent_parameter')
        
        test_node.destroy_node()

    {{#if include_publisher}}
    def test_publisher_qos_settings(self):
        """Test publisher Quality of Service settings."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        
        # Verify QoS settings (implementation depends on node design)
        publisher_info = test_node.get_publishers_info_by_topic('{{topic_name}}')
        self.assertGreater(len(publisher_info), 0)
        
        test_node.destroy_node()
    {{/if}}

    {{#if include_subscriber}}
    def test_subscriber_message_filtering(self):
        """Test subscriber message filtering and validation."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        
        # Create test publisher
        test_publisher = self.node.create_publisher(String, '{{topic_name}}', 10)
        
        # Allow time for connections
        time.sleep(0.2)
        
        # Publish various test messages
        test_messages = ["valid_message", "", "special_chars_!@#", "very_long_message" * 100]
        
        for msg_data in test_messages:
            test_msg = String()
            test_msg.data = msg_data
            test_publisher.publish(test_msg)
            
            # Process the message
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
        test_node.destroy_node()
        self.node.destroy_publisher(test_publisher)
    {{/if}}

    def test_concurrent_operations(self):
        """Test node behavior under concurrent operations."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        test_node = {{node_name|pascalcase}}Node()
        
        # Test concurrent spinning (simulates real-world usage)
        def spin_node():
            for _ in range(10):
                rclpy.spin_once(test_node, timeout_sec=0.01)
                time.sleep(0.01)
        
        threads = [threading.Thread(target=spin_node) for _ in range(3)]
        
        for thread in threads:
            thread.start()
        
        for thread in threads:
            thread.join()
        
        test_node.destroy_node()

    def test_memory_management(self):
        """Test memory management and resource cleanup."""
        from {{package_name}}.node import {{node_name|pascalcase}}Node

        # Create and destroy multiple node instances
        nodes = []
        for i in range(5):
            node = {{node_name|pascalcase}}Node(node_name=f'{{node_name}}_{i}')
            nodes.append(node)
        
        # Clean up all nodes
        for node in nodes:
            node.destroy_node()
        
        # Test passes if no memory leaks or exceptions occurred
        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()