# Python ROS 2 Package Template

The Python ROS 2 Package template generates flexible, prototype-friendly ROS 2 nodes using modern Python practices and the `rclpy` library.

## Overview

This template creates Python-based ROS 2 nodes with:
- **Modern Python**: Python 3.12+ with type hints and async support
- **Async/Await**: Native asyncio integration for concurrent operations
- **Rich Ecosystem**: Easy integration with scientific computing, ML, and web frameworks
- **Testing**: Comprehensive pytest integration
- **Documentation**: Auto-generated docstrings and type annotations

## Generated Structure

```
your_package/
├── package.xml                    # ROS 2 package manifest
├── setup.py                       # Python package configuration
├── setup.cfg                      # Python packaging metadata
├── README.md                      # Package documentation
├── CONTRIBUTING.md               # Development guidelines
├── Agents.md                      # AI interaction guide
├── your_package/
│   ├── __init__.py               # Package initialization
│   ├── your_package_node.py      # Main node implementation
│   └── utils.py                  # Utility functions
├── launch/
│   └── your_package_node.launch.py # Launch configuration
├── resource/
│   └── your_package               # Ament index resource
├── test/
│   ├── __init__.py               # Test package
│   └── test_your_package_node.py # Unit tests
└── docs/
    └── index.md                  # Package documentation
```

## Key Features

### Async/Await Support
Native asyncio integration for concurrent operations:
```python
import asyncio
import rclpy
from rclpy.node import Node

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')

    async def run_async_operation(self):
        """Example async method"""
        await asyncio.sleep(1.0)
        self.get_logger().info('Async operation completed')

    def timer_callback(self):
        """Timer callback that can spawn async tasks"""
        asyncio.create_task(self.run_async_operation())
```

### Type Hints and Documentation
Comprehensive type annotations and docstrings:
```python
from typing import Optional, List
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    """A ROS 2 node that publishes string messages.

    This node demonstrates basic publishing functionality with
    configurable message content and publishing rate.
    """

    def __init__(self, topic_name: str = 'chatter',
                 publish_rate: float = 1.0) -> None:
        """Initialize the publisher node.

        Args:
            topic_name: Name of the topic to publish to
            publish_rate: Publishing frequency in Hz
        """
        super().__init__('publisher_node')

        self.publisher: rclpy.publisher.Publisher[String] = self.create_publisher(
            String, topic_name, 10)

        self.timer: rclpy.timer.Timer = self.create_timer(
            1.0 / publish_rate, self.timer_callback)

        self.message_count: int = 0

    def timer_callback(self) -> None:
        """Publish a message on timer trigger."""
        msg = String()
        msg.data = f'Hello ROS 2! Message #{self.message_count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.message_count += 1
```

### pytest Integration
Comprehensive testing with ROS 2 fixtures:
```python
import pytest
import rclpy
from rclpy.node import Node
from your_package.your_package_node import PublisherNode

class TestPublisherNode:
    """Test cases for PublisherNode."""

    @pytest.fixture
    def node(self):
        """Fixture providing a PublisherNode instance."""
        rclpy.init()
        node = PublisherNode()
        yield node
        node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self, node):
        """Test that node is created successfully."""
        assert node.get_name() == 'publisher_node'
        assert node.publisher is not None

    def test_parameter_declaration(self, node):
        """Test parameter handling."""
        # Test parameter access and modification
        pass
```

## Usage Examples

### Basic Publisher
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Server
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Async Operations
```python
#!/usr/bin/env python3

import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AsyncPublisher(Node):
    def __init__(self):
        super().__init__('async_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """Timer callback that demonstrates async operations."""
        asyncio.create_task(self.publish_async())

    async def publish_async(self):
        """Async publishing method."""
        # Simulate async I/O operation
        await asyncio.sleep(0.1)

        msg = String()
        msg.data = 'Async message'
        self.publisher_.publish(msg)
        self.get_logger().info('Published async message')

def main(args=None):
    rclpy.init(args=args)
    node = AsyncPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Configuration Options

### Communication Patterns
- **Publishers**: Data broadcasting with configurable QoS
- **Subscribers**: Data reception with callback functions
- **Services**: Synchronous request-response communication
- **Actions**: Long-running task coordination
- **Parameters**: Dynamic configuration management

### Quality of Service (QoS)
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Best effort, volatile QoS for sensor data
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

# Reliable, transient local QoS for state data
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)
```

## Building and Testing

### Installation
```bash
# Install the package
pip install -e .

# Or using colcon
colcon build --packages-select your_package
source install/setup.bash
```

### Running Tests
```bash
# Using pytest directly
pytest

# Using colcon
colcon test --packages-select your_package
colcon test-result --verbose
```

### Running the Node
```bash
# Direct execution
python -m your_package.your_package_node

# Using ROS 2 launch
ros2 launch your_package your_package_node.launch.py

# Using ros2 run (if executable entry point is configured)
ros2 run your_package your_package_node
```

## Best Practices

### Performance Optimization
- Minimize global interpreter lock (GIL) impact
- Use numpy for numerical computations
- Consider multiprocessing for CPU-intensive tasks
- Use async operations for I/O-bound tasks

### Error Handling
```python
try:
    # ROS 2 operations
    pass
except rclpy.exceptions.ROSInterruptException:
    pass  # Expected during shutdown
except Exception as e:
    self.get_logger().error(f'Unexpected error: {e}')
    raise
```

### Logging
```python
# Use appropriate log levels
self.get_logger().debug('Detailed debugging information')
self.get_logger().info('General information messages')
self.get_logger().warn('Warning conditions')
self.get_logger().error('Error conditions')
self.get_logger().fatal('Fatal error conditions')
```

### Parameter Management
```python
# Declare parameters with defaults
self.declare_parameter('rate', 1.0)
self.declare_parameter('topic_name', 'default_topic')

# Access parameters
rate = self.get_parameter('rate').value
topic = self.get_parameter('topic_name').value

# Parameter callbacks for dynamic reconfiguration
def parameter_callback(self, params):
    for param in params:
        if param.name == 'rate':
            # Update rate logic
            pass
    return SetParametersResult(successful=True)

self.add_on_set_parameters_callback(self.parameter_callback)
```

## Integration Examples

### NumPy Integration
```python
import numpy as np
from sensor_msgs.msg import PointCloud2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2, 'pointcloud', self.process_pointcloud, 10)

    def process_pointcloud(self, msg):
        """Process point cloud data using NumPy."""
        # Convert ROS message to numpy array
        points = self.pointcloud2_to_numpy(msg)

        # Perform computations
        centroids = np.mean(points, axis=0)
        self.get_logger().info(f'Centroid: {centroids}')
```

### Web Framework Integration
```python
from flask import Flask
import threading

class WebEnabledNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.app = Flask(__name__)

        @self.app.route('/status')
        def status():
            return {'node_name': self.get_name(), 'status': 'running'}

        # Run Flask in separate thread
        self.web_thread = threading.Thread(target=self.app.run)
        self.web_thread.daemon = True
        self.web_thread.start()
```

## Troubleshooting

### Common Issues

#### Import Errors
**Symptoms:** `ModuleNotFoundError` when running
**Solution:** Ensure package is installed and Python path is correct

#### Async Operation Issues
**Symptoms:** Async operations not working as expected
**Solution:** Ensure proper event loop management and avoid blocking operations

#### Parameter Access Fails
**Symptoms:** Parameter values not updating
**Solution:** Check parameter declaration and callback registration

### Debug Tips
- Use `rclpy.logging` for detailed logging configuration
- Enable debug logging: `ros2 run your_package your_node --ros-args --log-level debug`
- Use Python debugger: `import pdb; pdb.set_trace()`
- Profile performance with `cProfile`

## Advanced Usage

### Multi-Threaded Executors
```python
import rclpy
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    node1 = Node1()
    node2 = Node2()

    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
```

### Custom Message Types
```python
# In setup.py, add message dependencies
# In package.xml, add <depend>your_custom_msgs</depend>

from your_custom_msgs.msg import CustomMessage

class CustomMessageNode(Node):
    def __init__(self):
        super().__init__('custom_node')
        self.publisher_ = self.create_publisher(CustomMessage, 'custom_topic', 10)
```

### Lifecycle Management
```python
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

class LifecycleExampleNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_node')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        # Initialize resources
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        # Start operations
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        # Stop operations
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        # Release resources
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        # Final cleanup
        return TransitionCallbackReturn.SUCCESS
```

## Migration from ROS 1

Key differences when migrating from ROS 1 Python nodes:
- Replace `rospy` with `rclpy`
- Use `Node` instead of node handles
- Update message/service type imports
- Use `rclpy.spin()` instead of `rospy.spin()`
- Update parameter access methods
- Use modern Python features and async patterns

## Contributing

To improve the Python template:

1. Add more async operation examples
2. Include additional testing patterns
3. Enhance type hint coverage
4. Add performance benchmarking
5. Include more integration examples

See the [contributing guide](CONTRIBUTING.md) for details on modifying templates.