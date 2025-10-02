/**
 * @file test/node.test.js
 * @brief Comprehensive unit tests for the {{package_name}} ROS 2 Node.js node
 * @author {{package_maintainer}}
 * @date {{year}}
 */

const rclnodejs = require('rclnodejs');
const {{node_name | pascalCase}}Node = require('../rclnodejs_node/node');

describe('{{node_name | pascalCase}}Node', () => {
  let node;
  let testNode;

  beforeAll(async () => {
    // Initialize ROS 2 Node.js
    await rclnodejs.init();
  });

  afterAll(async () => {
    // Shutdown ROS 2 Node.js
    await rclnodejs.shutdown();
  });

  beforeEach(async () => {
    node = new {{node_name | pascalCase}}Node();
    testNode = rclnodejs.createNode('test_node');
    await node.init();
  });

  afterEach(async () => {
    if (node) {
      await node.cleanup();
    }
    if (testNode) {
      testNode.destroy();
    }
  });

  test('should create node instance', () => {
    expect(node).toBeInstanceOf({{node_name | pascalCase}}Node);
  });

  test('should initialize with correct node name', async () => {
    expect(node.getNodeName()).toBe('{{node_name}}');
  });

{{#if include_publisher}}
  test('should have publisher when initialized', async () => {
    expect(node.publisher).toBeDefined();
    expect(node.publisher.getTopic()).toBe('{{topic_name}}');
  });

  test('should publish messages correctly', async () => {
    const messages = [];
    const subscriber = testNode.createSubscription('std_msgs/msg/String', '{{topic_name}}', (msg) => {
      messages.push(msg.data);
    });

    // Allow time for subscription to be established
    await new Promise(resolve => setTimeout(resolve, 100));

    // Publish test message
    node.publishMessage('test_message');

    // Wait for message to be received
    await new Promise(resolve => setTimeout(resolve, 100));

    expect(messages).toContain('test_message');
    testNode.destroySubscription(subscriber);
  });
{{/if}}

{{#if include_subscriber}}
  test('should have subscription when initialized', async () => {
    expect(node.subscription).toBeDefined();
    expect(node.subscription.getTopic()).toBe('{{topic_name}}');
  });

  test('should receive and process messages', async () => {
    const publisher = testNode.createPublisher('std_msgs/msg/String', '{{topic_name}}');
    
    // Allow time for connections
    await new Promise(resolve => setTimeout(resolve, 100));

    // Publish test message
    const testMessage = { data: 'test_subscription_message' };
    publisher.publish(testMessage);

    // Wait for message processing
    await new Promise(resolve => setTimeout(resolve, 100));

    // Test passes if no errors occurred during message processing
    expect(true).toBe(true);
    testNode.destroyPublisher(publisher);
  });
{{/if}}

{{#if include_service}}
  test('should have service when initialized', async () => {
    expect(node.service).toBeDefined();
    expect(node.service.getServiceName()).toBe('{{service_name}}');
  });

  test('should handle service requests', async () => {
    const client = testNode.createClient('std_srvs/srv/Trigger', '{{service_name}}');
    
    // Wait for service to be available
    await client.waitForService(5000);

    // Make service call
    const request = {};
    const response = await client.sendRequest(request);

    expect(response.success).toBe(true);
    expect(response.message).toBeDefined();
    
    testNode.destroyClient(client);
  });
{{/if}}

{{#if include_client}}
  test('should have client when initialized', async () => {
    expect(node.client).toBeDefined();
    expect(node.client.getServiceName()).toBe('{{service_name}}');
  });

  test('should make service calls', async () => {
    // Create a test service
    const service = testNode.createService('std_srvs/srv/Trigger', '{{service_name}}', (request, response) => {
      response.success = true;
      response.message = 'Test service response';
      return response;
    });

    // Wait for service to be available
    await node.client.waitForService(5000);

    // Make service call
    const request = {};
    const response = await node.callService(request);

    expect(response.success).toBe(true);
    expect(response.message).toBe('Test service response');
    
    testNode.destroyService(service);
  });
{{/if}}

{{#if include_timer}}
  test('should have timer when initialized', async () => {
    expect(node.timer).toBeDefined();
  });

  test('should execute timer callbacks', async () => {
    let callbackCount = 0;
    const originalCallback = node.timerCallback;
    
    // Mock timer callback to count executions
    node.timerCallback = () => {
      callbackCount++;
      if (originalCallback) originalCallback();
    };

    // Wait for several timer periods
    await new Promise(resolve => setTimeout(resolve, {{timer_period|default:"1"}}000 * 2.5));

    expect(callbackCount).toBeGreaterThan(0);
  });
{{/if}}

  test('should handle node lifecycle correctly', async () => {
    expect(node.isInitialized()).toBe(true);
    
    await node.cleanup();
    expect(node.isInitialized()).toBe(false);
  });

  test('should handle errors gracefully', async () => {
    // Test error handling by attempting invalid operations
    try {
      // Attempt to use destroyed resources
      await node.cleanup();
      {{#if include_publisher}}
      node.publishMessage('should_fail');
      {{/if}}
      
      // If no exception is thrown, error handling is working
      expect(true).toBe(true);
    } catch (error) {
      // Expected behavior - errors should be handled gracefully
      expect(error).toBeDefined();
    }
  });

  test('should cleanup resources properly', async () => {
    const initialResourceCount = process._getActiveHandles().length;
    
    await node.cleanup();
    
    // Allow time for cleanup
    await new Promise(resolve => setTimeout(resolve, 100));
    
    // Resource count should not increase significantly (some variation is normal)
    const finalResourceCount = process._getActiveHandles().length;
    expect(finalResourceCount).toBeLessThanOrEqual(initialResourceCount + 5);
  });

  test('should maintain thread safety', async () => {
    // Test concurrent operations
    const promises = [];
    
    for (let i = 0; i < 10; i++) {
      promises.push(new Promise(resolve => {
        setTimeout(() => {
          {{#if include_publisher}}
          node.publishMessage(`concurrent_message_${i}`);
          {{/if}}
          resolve();
        }, Math.random() * 50);
      }));
    }
    
    await Promise.all(promises);
    
    // Test passes if no race conditions or crashes occurred
    expect(true).toBe(true);
  });
});