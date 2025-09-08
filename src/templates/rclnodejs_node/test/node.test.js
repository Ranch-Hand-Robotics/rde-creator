/**
 * @file test/node.test.js
 * @brief Unit tests for the {{package_name}} ROS 2 Node.js node
 */

const {{node_name | pascalCase}}Node = require('../node');

describe('{{node_name | pascalCase}}Node', () => {
  let node;

  beforeEach(() => {
    node = new {{node_name | pascalCase}}Node();
  });

  afterEach(() => {
    // Cleanup after each test
  });

  test('should create node instance', () => {
    expect(node).toBeInstanceOf({{node_name | pascalCase}}Node);
  });

{{#if include_publisher}}
  test('should have publisher when initialized', async () => {
    await node.init();
    expect(node.publisher).toBeDefined();
  });
{{/if}}

{{#if include_subscriber}}
  test('should have subscription when initialized', async () => {
    await node.init();
    expect(node.subscription).toBeDefined();
  });
{{/if}}

{{#if include_service}}
  test('should have service when initialized', async () => {
    await node.init();
    expect(node.service).toBeDefined();
  });
{{/if}}

{{#if include_client}}
  test('should have client when initialized', async () => {
    await node.init();
    expect(node.client).toBeDefined();
  });
{{/if}}

{{#if include_timer}}
  test('should have timer when initialized', async () => {
    await node.init();
    expect(node.timer).toBeDefined();
  });
{{/if}}

  test('should cleanup resources', () => {
    node.cleanup();
    // Add assertions for cleanup
  });
});