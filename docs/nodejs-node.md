# Node.js ROS 2 Node Template

The Node.js ROS 2 Node template generates ROS 2 nodes using the `rclnodejs` library, enabling web integration and asynchronous operations in JavaScript/TypeScript.

## Overview

This template creates Node.js-based ROS 2 nodes with:
- **Full rclnodejs Integration**: Complete ROS 2 functionality in JavaScript
- **Promise-Based Async**: Modern async/await patterns
- **Web Capabilities**: HTTP server integration and REST APIs
- **TypeScript Support**: Optional TypeScript with type definitions
- **Testing**: Jest integration with ROS 2 mocks
- **Event-Driven**: Native Node.js event loop integration

## Generated Structure

```
your_package/
├── package.json                   # Node.js package configuration
├── tsconfig.json                 # TypeScript configuration (optional)
├── package.xml                   # ROS 2 package manifest
├── README.md                     # Package documentation
├── CONTRIBUTING.md               # Development guidelines
├── Agents.md                     # AI interaction guide
├── lib/
│   ├── index.js                  # Main entry point
│   ├── your_package_node.js      # Main node implementation
│   └── utils.js                  # Utility functions
├── launch/
│   └── your_package_node.launch.py # Launch configuration
├── resource/
│   └── your_package               # Ament index resource
├── test/
│   ├── your_package_node.test.js # Unit tests
│   └── __mocks__/                # Jest mocks
└── docs/
    └── api.md                    # API documentation
```

## Key Features

### Promise-Based Async Operations
Native Promise support for ROS 2 operations:
```javascript
const rclnodejs = require('rclnodejs');

class AsyncNode extends rclnodejs.Node {
  constructor() {
    super('async_node');

    // Async service client
    this.client = this.createClient(
      'example_interfaces/srv/AddTwoInts',
      'add_two_ints'
    );

    // Async publisher
    this.publisher = this.createPublisher(
      'std_msgs/msg/String',
      'chatter'
    );

    // Timer with async callback
    this.timer = this.createTimer(1000, async () => {
      await this.publishMessage();
    });
  }

  async publishMessage() {
    const msg = {
      data: `Hello ROS 2! Time: ${Date.now()}`
    };

    await this.publisher.publish(msg);
    this.getLogger().info(`Published: ${msg.data}`);
  }

  async callService(a, b) {
    const request = { a, b };

    try {
      const response = await this.client.sendRequest(request);
      this.getLogger().info(`Result: ${a} + ${b} = ${response.sum}`);
      return response.sum;
    } catch (error) {
      this.getLogger().error(`Service call failed: ${error}`);
      throw error;
    }
  }
}
```

### Web Integration
Built-in HTTP server capabilities:
```javascript
const express = require('express');
const rclnodejs = require('rclnodejs');

class WebEnabledNode extends rclnodejs.Node {
  constructor() {
    super('web_node');

    // Express app for REST API
    this.app = express();
    this.app.use(express.json());

    // ROS 2 service
    this.service = this.createService(
      'std_srvs/srv/Trigger',
      'trigger_service',
      this.handleTrigger.bind(this)
    );

    // REST endpoints
    this.app.get('/status', (req, res) => {
      res.json({
        node_name: this.name(),
        status: 'running',
        services: this.getServiceNamesAndTypes()
      });
    });

    this.app.post('/trigger', async (req, res) => {
      try {
        // Call ROS 2 service
        const result = await this.callTriggerService();
        res.json({ success: true, result });
      } catch (error) {
        res.status(500).json({ error: error.message });
      }
    });

    // Start web server
    this.server = this.app.listen(3000, () => {
      this.getLogger().info('Web server listening on port 3000');
    });
  }

  async handleTrigger(request, response) {
    this.getLogger().info('Trigger service called');

    // Perform async operation
    await this.performAsyncTask();

    return {
      success: true,
      message: 'Trigger executed successfully'
    };
  }

  destroy() {
    if (this.server) {
      this.server.close();
    }
    super.destroy();
  }
}
```

### TypeScript Support
Optional TypeScript with full type safety:
```typescript
import * as rclnodejs from 'rclnodejs';

interface PublisherConfig {
  topicName: string;
  messageType: string;
  publishRate: number;
}

class TypedPublisher extends rclnodejs.Node {
  private publisher: rclnodejs.Publisher<any>;
  private timer: rclnodejs.Timer;
  private config: PublisherConfig;

  constructor(config: PublisherConfig) {
    super('typed_publisher');
    this.config = config;

    this.publisher = this.createPublisher(
      config.messageType,
      config.topicName
    );

    this.timer = this.createTimer(
      1000 / config.publishRate,
      this.publishCallback.bind(this)
    );
  }

  private publishCallback(): void {
    const message = this.createMessage();
    this.publisher.publish(message);
    this.getLogger().info(`Published to ${this.config.topicName}`);
  }

  private createMessage(): any {
    // Type-safe message creation
    return {
      data: `Message at ${new Date().toISOString()}`,
      timestamp: Date.now()
    };
  }
}
```

## Usage Examples

### Basic Publisher
```javascript
const rclnodejs = require('rclnodejs');

class MinimalPublisher extends rclnodejs.Node {
  constructor() {
    super('minimal_publisher');

    this.publisher = this.createPublisher(
      'std_msgs/msg/String',
      'topic'
    );

    this.timer = this.createTimer(500, () => {
      this.publishMessage();
    });

    this.messageCount = 0;
  }

  publishMessage() {
    const msg = {
      data: `Hello World: ${this.messageCount}`
    };

    this.publisher.publish(msg);
    this.getLogger().info(`Publishing: "${msg.data}"`);
    this.messageCount++;
  }
}

async function main() {
  await rclnodejs.init();
  const node = new MinimalPublisher();
  rclnodejs.spin(node);
}

main();
```

### Service Server
```javascript
const rclnodejs = require('rclnodejs');

class MinimalService extends rclnodejs.Node {
  constructor() {
    super('minimal_service');

    this.service = this.createService(
      'example_interfaces/srv/AddTwoInts',
      'add_two_ints',
      this.addCallback.bind(this)
    );
  }

  addCallback(request, response) {
    this.getLogger().info(
      `Incoming request: ${request.a} + ${request.b}`
    );

    response.sum = request.a + request.b;
    return response;
  }
}

async function main() {
  await rclnodejs.init();
  const node = new MinimalService();
  rclnodejs.spin(node);
}

main();
```

### Action Client
```javascript
const rclnodejs = require('rclnodejs');

class FibonacciActionClient extends rclnodejs.Node {
  constructor() {
    super('fibonacci_action_client');

    this.actionClient = this.createActionClient(
      'example_interfaces/action/Fibonacci',
      'fibonacci'
    );

    this.actionClient.waitForServer().then(() => {
      this.sendGoal();
    });
  }

  async sendGoal() {
    const goal = { order: 10 };

    try {
      const goalHandle = await this.actionClient.sendGoal(goal);

      // Handle feedback
      goalHandle.on('feedback', (feedback) => {
        this.getLogger().info(`Feedback: ${feedback.sequence}`);
      });

      // Wait for result
      const result = await goalHandle.getResult();
      this.getLogger().info(`Result: ${result.sequence}`);

    } catch (error) {
      this.getLogger().error(`Action failed: ${error}`);
    }
  }
}
```

## Configuration Options

### Communication Patterns
- **Publishers**: Data broadcasting with QoS settings
- **Subscribers**: Event-driven data reception
- **Services**: Request-response with Promise support
- **Actions**: Long-running task coordination
- **Parameters**: Dynamic configuration management

### Quality of Service (QoS)
```javascript
// Define QoS profiles
const sensorQoS = {
  reliability: rclnodejs.QoS.ReliabilityPolicy.BEST_EFFORT,
  durability: rclnodejs.QoS.DurabilityPolicy.VOLATILE,
  depth: 10
};

const stateQoS = {
  reliability: rclnodejs.QoS.ReliabilityPolicy.RELIABLE,
  durability: rclnodejs.QoS.DurabilityPolicy.TRANSIENT_LOCAL,
  depth: 1
};

// Use with publishers/subscribers
this.publisher = this.createPublisher('sensor_msgs/msg/LaserScan', 'scan', sensorQoS);
this.subscriber = this.createSubscription('nav_msgs/msg/Odometry', 'odom', stateQoS, callback);
```

## Building and Testing

### Installation
```bash
# Install dependencies
npm install

# Build TypeScript (if used)
npm run build

# Install ROS 2 package
colcon build --packages-select your_package
source install/setup.bash
```

### Running Tests
```bash
# Run Jest tests
npm test

# With coverage
npm run test:coverage

# Using colcon
colcon test --packages-select your_package
```

### Running the Node
```bash
# Direct execution
node lib/index.js

# Using ROS 2 launch
ros2 launch your_package your_package_node.launch.py

# With environment variables
NODE_ENV=production node lib/index.js
```

## Best Practices

### Error Handling
```javascript
class RobustNode extends rclnodejs.Node {
  constructor() {
    super('robust_node');

    // Handle async errors
    this.setupServices().catch(error => {
      this.getLogger().error(`Setup failed: ${error}`);
      process.exit(1);
    });
  }

  async setupServices() {
    try {
      this.service = this.createService(
        'std_srvs/srv/Trigger',
        'my_service',
        this.handleService.bind(this)
      );
    } catch (error) {
      this.getLogger().error(`Service creation failed: ${error}`);
      throw error;
    }
  }

  handleService(request, response) {
    try {
      // Service logic with error handling
      if (!this.validateRequest(request)) {
        return {
          success: false,
          message: 'Invalid request'
        };
      }

      return {
        success: true,
        message: 'Service executed successfully'
      };
    } catch (error) {
      this.getLogger().error(`Service error: ${error}`);
      return {
        success: false,
        message: `Internal error: ${error.message}`
      };
    }
  }

  validateRequest(request) {
    // Validation logic
    return true;
  }
}
```

### Memory Management
```javascript
class MemoryEfficientNode extends rclnodejs.Node {
  constructor() {
    super('memory_efficient_node');

    // Use object pooling for frequently created objects
    this.messagePool = [];
    this.maxPoolSize = 10;

    this.publisher = this.createPublisher(
      'std_msgs/msg/String',
      'topic'
    );

    this.timer = this.createTimer(100, () => {
      const msg = this.getMessageFromPool();
      msg.data = `Message ${Date.now()}`;
      this.publisher.publish(msg);
      this.returnMessageToPool(msg);
    });
  }

  getMessageFromPool() {
    return this.messagePool.pop() || { data: '' };
  }

  returnMessageToPool(message) {
    if (this.messagePool.length < this.maxPoolSize) {
      // Reset message
      message.data = '';
      this.messagePool.push(message);
    }
  }
}
```

### Logging
```javascript
class WellLoggedNode extends rclnodejs.Node {
  constructor() {
    super('well_logged_node');

    // Structured logging
    this.logger = {
      debug: (msg, meta = {}) => this.log('DEBUG', msg, meta),
      info: (msg, meta = {}) => this.log('INFO', msg, meta),
      warn: (msg, meta = {}) => this.log('WARN', msg, meta),
      error: (msg, meta = {}) => this.log('ERROR', msg, meta)
    };
  }

  log(level, message, metadata = {}) {
    const logEntry = {
      level,
      message,
      timestamp: new Date().toISOString(),
      node: this.name(),
      ...metadata
    };

    this.getLogger().info(JSON.stringify(logEntry));
  }

  async performOperation() {
    const startTime = Date.now();

    try {
      this.logger.debug('Starting operation', { operation: 'performOperation' });

      // Operation logic
      await this.doSomething();

      const duration = Date.now() - startTime;
      this.logger.info('Operation completed', {
        operation: 'performOperation',
        duration_ms: duration,
        success: true
      });

    } catch (error) {
      const duration = Date.now() - startTime;
      this.logger.error('Operation failed', {
        operation: 'performOperation',
        duration_ms: duration,
        error: error.message,
        success: false
      });
      throw error;
    }
  }
}
```

## Testing

### Jest Integration
```javascript
const rclnodejs = require('rclnodejs');

describe('PublisherNode', () => {
  let node;

  beforeAll(async () => {
    await rclnodejs.init();
  });

  afterAll(() => {
    rclnodejs.shutdown();
  });

  beforeEach(() => {
    node = new PublisherNode();
  });

  afterEach(() => {
    node.destroy();
  });

  test('should create publisher', () => {
    expect(node.publisher).toBeDefined();
    expect(node.publisher.topicName).toBe('chatter');
  });

  test('should publish messages', (done) => {
    const mockCallback = jest.fn();
    const subscription = node.createSubscription(
      'std_msgs/msg/String',
      'chatter',
      mockCallback
    );

    // Wait for message
    setTimeout(() => {
      expect(mockCallback).toHaveBeenCalled();
      subscription.destroy();
      done();
    }, 100);
  });
});
```

### Mocking ROS 2 Components
```javascript
// __mocks__/rclnodejs.js
const mockPublisher = {
  publish: jest.fn(),
  destroy: jest.fn()
};

const mockSubscriber = {
  destroy: jest.fn()
};

const mockService = {
  destroy: jest.fn()
};

module.exports = {
  init: jest.fn().mockResolvedValue(),
  shutdown: jest.fn(),
  createNode: jest.fn().mockReturnValue({
    createPublisher: jest.fn().mockReturnValue(mockPublisher),
    createSubscriber: jest.fn().mockReturnValue(mockSubscriber),
    createService: jest.fn().mockReturnValue(mockService),
    destroy: jest.fn()
  }),
  QoS: {
    ReliabilityPolicy: {
      RELIABLE: 'RELIABLE',
      BEST_EFFORT: 'BEST_EFFORT'
    }
  }
};
```

## Advanced Usage

### WebSocket Integration
```javascript
const WebSocket = require('ws');
const rclnodejs = require('rclnodejs');

class WebSocketNode extends rclnodejs.Node {
  constructor() {
    super('websocket_node');

    // ROS 2 subscriber
    this.subscriber = this.createSubscription(
      'sensor_msgs/msg/Imu',
      'imu',
      this.handleImuData.bind(this)
    );

    // WebSocket server
    this.wss = new WebSocket.Server({ port: 8080 });

    this.wss.on('connection', (ws) => {
      this.getLogger().info('WebSocket client connected');

      ws.on('message', (message) => {
        this.handleWebSocketMessage(ws, message);
      });
    });
  }

  handleImuData(msg) {
    // Broadcast IMU data to all WebSocket clients
    const data = JSON.stringify({
      type: 'imu',
      data: msg
    });

    this.wss.clients.forEach(client => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(data);
      }
    });
  }

  handleWebSocketMessage(ws, message) {
    try {
      const data = JSON.parse(message);

      if (data.type === 'command') {
        // Handle commands from web clients
        this.processCommand(data.command);
      }
    } catch (error) {
      this.getLogger().error(`Invalid WebSocket message: ${error}`);
    }
  }

  destroy() {
    if (this.wss) {
      this.wss.close();
    }
    super.destroy();
  }
}
```

### REST API with Express
```javascript
const express = require('express');
const rclnodejs = require('rclnodejs');

class RESTNode extends rclnodejs.Node {
  constructor() {
    super('rest_node');

    this.app = express();
    this.app.use(express.json());

    // ROS 2 services
    this.getStatusService = this.createService(
      'std_srvs/srv/Trigger',
      'get_status',
      this.handleGetStatus.bind(this)
    );

    this.setParameterService = this.createService(
      'rcl_interfaces/srv/SetParameters',
      'set_parameters',
      this.handleSetParameters.bind(this)
    );

    // REST endpoints
    this.setupRoutes();

    // Start server
    this.server = this.app.listen(3000, () => {
      this.getLogger().info('REST API listening on port 3000');
    });
  }

  setupRoutes() {
    // Get node status
    this.app.get('/status', async (req, res) => {
      try {
        const status = await this.getNodeStatus();
        res.json(status);
      } catch (error) {
        res.status(500).json({ error: error.message });
      }
    });

    // Set parameters
    this.app.post('/parameters', async (req, res) => {
      try {
        const result = await this.setParameters(req.body);
        res.json(result);
      } catch (error) {
        res.status(400).json({ error: error.message });
      }
    });

    // Get topics
    this.app.get('/topics', (req, res) => {
      const topics = this.getTopicNamesAndTypes();
      res.json({ topics });
    });
  }

  async getNodeStatus() {
    return {
      name: this.name(),
      namespace: this.namespace(),
      publishers: this.getPublisherNamesAndTypes(),
      subscribers: this.getSubscriberNamesAndTypes(),
      services: this.getServiceNamesAndTypes(),
      uptime: process.uptime()
    };
  }

  destroy() {
    if (this.server) {
      this.server.close();
    }
    super.destroy();
  }
}
```

### Integration with Databases
```javascript
const { MongoClient } = require('mongodb');
const rclnodejs = require('rclnodejs');

class DatabaseNode extends rclnodejs.Node {
  constructor() {
    super('database_node');

    this.mongoClient = null;
    this.database = null;

    // Initialize database connection
    this.initDatabase();

    // ROS 2 subscriber for data storage
    this.subscriber = this.createSubscription(
      'sensor_msgs/msg/PointCloud2',
      'pointcloud',
      this.handlePointCloud.bind(this)
    );
  }

  async initDatabase() {
    try {
      this.mongoClient = new MongoClient('mongodb://localhost:27017');
      await this.mongoClient.connect();
      this.database = this.mongoClient.db('ros_data');

      this.getLogger().info('Connected to MongoDB');
    } catch (error) {
      this.getLogger().error(`Database connection failed: ${error}`);
    }
  }

  async handlePointCloud(msg) {
    if (!this.database) {
      this.getLogger().warn('Database not connected, skipping data storage');
      return;
    }

    try {
      const collection = this.database.collection('pointclouds');

      const document = {
        timestamp: new Date(),
        topic: 'pointcloud',
        data: msg  // Store the full ROS message
      };

      await collection.insertOne(document);
      this.getLogger().info('Point cloud data stored in database');

    } catch (error) {
      this.getLogger().error(`Database insertion failed: ${error}`);
    }
  }

  destroy() {
    if (this.mongoClient) {
      this.mongoClient.close();
    }
    super.destroy();
  }
}
```

## Troubleshooting

### Common Issues

#### rclnodejs Not Found
**Symptoms:** `Cannot find module 'rclnodejs'`
**Solution:** Install rclnodejs: `npm install rclnodejs`

#### ROS 2 Initialization Fails
**Symptoms:** `rclnodejs.init()` throws error
**Solution:** Ensure ROS 2 environment is sourced and rclnodejs is built

#### Web Server Port Conflicts
**Symptoms:** `EADDRINUSE` error
**Solution:** Change port number or free the port

#### Memory Leaks
**Symptoms:** Increasing memory usage over time
**Solution:** Ensure proper cleanup of timers, subscriptions, and object pools

### Debug Tips
- Use `console.log()` for debugging (maps to ROS logging)
- Enable verbose logging: `rclnodejs.setLoggerLevel(rclnodejs.LoggingSeverity.DEBUG)`
- Use Node.js debugger: `node --inspect lib/index.js`
- Monitor with `htop` or `node --prof` for performance issues

## Performance Considerations

### Event Loop Blocking
```javascript
// Bad: Blocks event loop
this.timer = this.createTimer(1000, () => {
  const result = this.blockingOperation(); // Blocks for seconds
  this.publisher.publish(result);
});

// Good: Use async operations
this.timer = this.createTimer(1000, async () => {
  const result = await this.asyncOperation(); // Non-blocking
  this.publisher.publish(result);
});
```

### Memory Management
- Use object pooling for frequently created objects
- Avoid closures in hot paths
- Monitor heap usage with `--expose-gc`
- Use streams for large data processing

### Scaling Considerations
- Consider clustering for CPU-intensive tasks
- Use worker threads for blocking operations
- Implement connection pooling for databases
- Use Redis for inter-process communication

## Migration from ROS 1

Key differences when migrating from ROS 1 JavaScript nodes:
- Replace `rosnodejs` with `rclnodejs`
- Update message/service type definitions
- Use Promises instead of callbacks where possible
- Update QoS settings syntax
- Use modern JavaScript features (async/await, destructuring)
- Update parameter access methods

## Contributing

To improve the Node.js template:
1. Add more web integration examples
2. Include additional testing patterns
3. Enhance TypeScript support
4. Add performance benchmarks
5. Include more database integration examples

See the [contributing guide](CONTRIBUTING.md) for details on modifying templates.