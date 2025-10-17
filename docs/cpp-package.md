# C++ ROS 2 Package Template

The C++ ROS 2 Package template generates high-performance, composable ROS 2 nodes using modern C++ standards and best practices.

## Overview

This template creates production-ready ROS 2 nodes with:

- **Modern C++**: C++17+ features with RAII principles
- **Component Architecture**: Full `rclcpp_components` support
- **Cross-Platform**: Windows, Linux, and macOS compatibility
- **Lifecycle Management**: Optional `rclcpp_lifecycle` integration
- **Testing**: Integrated gtest framework
- **Visibility Control**: Proper symbol visibility management

## Generated Structure

```
your_package/
├── CMakeLists.txt                 # Modern Cmake with ROS distro detection
├── package.xml                    # ROS 2 package manifest
├── README.md                      # Package documentation
├── CONTRIBUTING.md               # Development guidelines
├── Agents.md                      # AI interaction guide
├── include/
│   └── your_package/
│       ├── your_package.hpp       # Main node header
│       └── your_package_visibility_control.h  # Visibility macros
├── src/
│   ├── your_package.cpp           # Main node implementation
│   └── your_package_component.cpp # Component registration
├── launch/
│   └── your_package_node.launch.py # Launch configuration
├── resource/
│   └── your_package               # Ament index resource
└── test/
    └── test_your_package.cpp      # Unit tests
```

## Key Features

### Component-Based Architecture
Generated nodes are fully composable:
```cpp
// Component registration
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(your_package::YourPackageNode);
```

### Visibility Control
Cross-platform symbol visibility management:
```cpp
// Visibility control header
#ifndef YOUR_PACKAGE__VISIBILITY_CONTROL_H_
#define YOUR_PACKAGE__VISIBILITY_CONTROL_H_

// Platform-specific visibility macros
#endif

// Class declaration
class YOUR_PACKAGE_PUBLIC YourPackageNode : public rclcpp::Node {
  // ...
};
```

### Modern CMake
Automatic ROS distribution detection:
```cmake
# Runtime ROS distro detection
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(ROS_DISTRO STREQUAL "humble" OR ROS_DISTRO STREQUAL "galactic" OR ROS_DISTRO STREQUAL "foxy")
  # Legacy ament_target_dependencies approach
  ament_target_dependencies(your_package_component rclcpp rclcpp_components)
else()
  # Modern target_link_libraries approach
  target_link_libraries(your_package_component PUBLIC rclcpp::rclcpp)
endif()
```

## Usage Examples

### Basic Publisher Node
```cpp
class PublisherNode : public rclcpp::Node {
public:
  PublisherNode() : Node("publisher_node") {
    publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PublisherNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2!";
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

### Service Server
```cpp
class ServiceNode : public rclcpp::Node {
public:
  ServiceNode() : Node("service_node") {
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&ServiceNode::handle_service, this, _1, _2, _3));
  }

private:
  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {

    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(), "Incoming request: %ld + %ld = %ld",
                request->a, request->b, response->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
```

## Configuration Options

### Lifecycle Support
Enable lifecycle management for nodes that need state transitions:
- **Configure**: Parameter validation and resource allocation
- **Activate**: Start publishing/subscribing
- **Deactivate**: Pause operations
- **Cleanup**: Release resources
- **Shutdown**: Final cleanup

### Communication Patterns
- **Publishers**: Data broadcasting
- **Subscribers**: Data reception with callbacks
- **Services**: Request-response communication
- **Actions**: Long-running task coordination
- **Parameters**: Dynamic configuration

## Building and Testing

### Build Process
```bash
# Standard colcon build
colcon build --packages-select your_package

# With tests
colcon build --packages-select your_package --cmake-args -DBUILD_TESTING=ON
```

### Running Tests
```bash
# Run tests
colcon test --packages-select your_package

# View results
colcon test-result --verbose
```

### Component Loading
```bash
# Load component at runtime
ros2 component load /ComponentManager your_package YourPackageNode
```

## Best Practices

### Performance Optimization
- Use appropriate QoS settings for your use case
- Minimize dynamic memory allocation in hot paths
- Use `std::chrono` for timing instead of deprecated methods
- Leverage C++17 features like `constexpr` and `if constexpr`

### Error Handling
- Use ROS 2 logging macros (`RCLCPP_*`)
- Check return values from ROS 2 APIs
- Implement proper exception safety
- Provide meaningful error messages

### Thread Safety
- ROS 2 nodes are generally single-threaded
- Use mutexes for shared data access
- Consider using `rclcpp::executors::MultiThreadedExecutor` for concurrent operations

## Troubleshooting

### Common Issues

#### Visibility Errors
**Symptoms:** Linker errors about missing symbols
**Solution:** Ensure `YOUR_PACKAGE_BUILDING_LIBRARY` is defined in CMakeLists.txt

#### Component Not Found
**Symptoms:** `ros2 component list` doesn't show your component
**Solution:** Check that `RCLCPP_COMPONENTS_REGISTER_NODE` is called and library is installed

#### Test Discovery Fails
**Symptoms:** Tests not found by `colcon test`
**Solution:** Ensure test files contain proper `TEST()` or `TEST_F()` macros

### Debug Tips
- Use `RCLCPP_DEBUG` for detailed logging
- Enable ROS 2 debug logging: `ros2 run your_package your_node --ros-args --log-level debug`
- Use `gdb` or `lldb` for native debugging
- Check component loading with verbose output

## Advanced Usage

### Custom Message Types
Add custom message dependencies to `package.xml` and `CMakeLists.txt`:
```xml
<!-- package.xml -->
<depend>your_custom_msgs</depend>
```

```cmake
# CMakeLists.txt
find_package(your_custom_msgs REQUIRED)
ament_target_dependencies(your_package_component your_custom_msgs)
```

### Parameter Management
```cpp
// Declare parameters
declare_parameter("rate", 10.0);
declare_parameter("topic_name", "default_topic");

// Use parameters
double rate = get_parameter("rate").as_double();
std::string topic = get_parameter("topic_name").as_string();
```

### Lifecycle Integration
```cpp
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode {
  // Implement lifecycle callbacks
  LifecycleNodeReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleNodeReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleNodeReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleNodeReturn on_shutdown(const rclcpp_lifecycle::State &) override;
};
```

## Migration from ROS 1

If migrating from ROS 1 C++ nodes:
- Replace `ros::init()` with `rclcpp::init()`
- Use `rclcpp::Node` instead of `ros::NodeHandle`
- Replace ROS 1 message types with ROS 2 equivalents
- Update CMakeLists.txt to use `ament_cmake`
- Use modern C++ features and smart pointers

## Contributing

To improve this template:

1. Test with different ROS 2 distributions
2. Verify cross-platform compatibility
3. Add performance benchmarks
4. Include more communication pattern examples
5. Enhance error handling and logging

See the [contributing guide](CONTRIBUTING.md) for details on modifying templates.