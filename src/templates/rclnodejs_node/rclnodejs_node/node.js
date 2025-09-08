#!/usr/bin/env node

/**
 * @file node.js
 * @brief Main ROS 2 Node.js node implementation using rclnodejs
 *
 * This file contains the implementation of the {{node_name}} ROS 2 node.
 * It demonstrates various ROS 2 communication patterns using rclnodejs.
 */

'use strict';

const rclnodejs = require('rclnodejs');

/**
 * Main ROS 2 Node class
 */
class {{node_name | pascalCase}}Node {
  /**
   * Constructor for the ROS 2 node
   */
  constructor() {
    this.node = null;
{{#if include_publisher}}
    this.publisher = null;
{{/if}}
{{#if include_subscriber}}
    this.subscription = null;
{{/if}}
{{#if include_service}}
    this.service = null;
{{/if}}
{{#if include_client}}
    this.client = null;
{{/if}}
{{#if include_timer}}
    this.timer = null;
{{/if}}
  }

  /**
   * Initialize the ROS 2 node
   */
  async init() {
    // Initialize rclnodejs
    await rclnodejs.init();

    // Create the ROS 2 node
    this.node = rclnodejs.createNode('{{node_name}}');

    console.log('{{node_name}} node initialized');

{{#if include_publisher}}
    // Create publisher
    this.publisher = this.node.createPublisher('std_msgs/msg/String', '{{topic_name}}');
    console.log('Publisher created for topic: {{topic_name}}');
{{/if}}

{{#if include_subscriber}}
    // Create subscriber
    this.subscription = this.node.createSubscription(
      'std_msgs/msg/String',
      '{{topic_name}}',
      (msg) => {
        console.log(`Received message: ${msg.data}`);
        this.processMessage(msg);
      }
    );
    console.log('Subscriber created for topic: {{topic_name}}');
{{/if}}

{{#if include_service}}
    // Create service server
    this.service = this.node.createService(
      'example_interfaces/srv/AddTwoInts',
      '{{service_name}}',
      (request, response) => {
        console.log(`Received service request: ${request.a} + ${request.b}`);
        response.sum = request.a + request.b;
        return response;
      }
    );
    console.log('Service server created for service: {{service_name}}');
{{/if}}

{{#if include_client}}
    // Create service client
    this.client = this.node.createClient('example_interfaces/srv/AddTwoInts', '{{service_name}}');
    console.log('Service client created for service: {{service_name}}');
{{/if}}

{{#if include_timer}}
    // Create timer
    this.timer = this.node.createTimer({{timer_period}} * 1000, () => {
      this.timerCallback();
    });
    console.log('Timer created with period: {{timer_period}}s');
{{/if}}

{{#if include_parameters}}
    // Declare parameters
    this.node.declareParameter('example_param', 'default_value');
    console.log('Parameter declared: example_param');
{{/if}}
  }

{{#if include_publisher}}
  /**
   * Publish a message
   * @param {string} message - Message to publish
   */
  publishMessage(message) {
    if (this.publisher) {
      const msg = rclnodejs.createMessageObject('std_msgs/msg/String');
      msg.data = message;
      this.publisher.publish(msg);
      console.log(`Published message: ${message}`);
    }
  }
{{/if}}

{{#if include_subscriber}}
  /**
   * Process received message
   * @param {Object} msg - Received message
   */
  processMessage(msg) {
    // Process the received message
    console.log(`Processing message: ${msg.data}`);

    // Add your message processing logic here
  }
{{/if}}

{{#if include_client}}
  /**
   * Call service
   * @param {number} a - First number
   * @param {number} b - Second number
   */
  async callService(a, b) {
    if (this.client) {
      const request = rclnodejs.createMessageObject('example_interfaces/srv/AddTwoInts_Request');
      request.a = a;
      request.b = b;

      try {
        const response = await this.client.call(request);
        console.log(`Service response: ${a} + ${b} = ${response.sum}`);
        return response.sum;
      } catch (error) {
        console.error('Service call failed:', error);
      }
    }
  }
{{/if}}

{{#if include_timer}}
  /**
   * Timer callback function
   */
  timerCallback() {
    console.log('Timer callback executed');

{{#if include_publisher}}
    // Publish a message on timer
    const timestamp = new Date().toISOString();
    this.publishMessage(`Timer message at ${timestamp}`);
{{/if}}

    // Add your periodic task logic here
  }
{{/if}}

{{#if include_parameters}}
  /**
   * Get parameter value
   * @param {string} paramName - Parameter name
   * @returns {any} Parameter value
   */
  getParameter(paramName) {
    try {
      return this.node.getParameterValue(paramName);
    } catch (error) {
      console.error(`Failed to get parameter ${paramName}:`, error);
      return null;
    }
  }

  /**
   * Set parameter value
   * @param {string} paramName - Parameter name
   * @param {any} value - Parameter value
   */
  setParameter(paramName, value) {
    try {
      this.node.setParameterValue(paramName, value);
      console.log(`Parameter ${paramName} set to: ${value}`);
    } catch (error) {
      console.error(`Failed to set parameter ${paramName}:`, error);
    }
  }
{{/if}}

  /**
   * Run the node
   */
  async run() {
    console.log('{{node_name}} node is running...');

    // Spin the node
    rclnodejs.spin(this.node);

    // Handle graceful shutdown
    process.on('SIGINT', () => {
      console.log('Shutting down {{node_name}} node...');
      this.cleanup();
      process.exit(0);
    });

    process.on('SIGTERM', () => {
      console.log('Shutting down {{node_name}} node...');
      this.cleanup();
      process.exit(0);
    });
  }

  /**
   * Cleanup resources
   */
  cleanup() {
    if (this.node) {
      this.node.destroy();
      console.log('Node destroyed');
    }

    // Shutdown rclnodejs
    rclnodejs.shutdown();
  }
}

// Main execution
async function main() {
  try {
    const node = new {{node_name | pascalCase}}Node();
    await node.init();
    await node.run();
  } catch (error) {
    console.error('Error:', error);
    process.exit(1);
  }
}

// Run the main function
if (require.main === module) {
  main();
}

module.exports = {{node_name | pascalCase}}Node;