# Resource Package Template

The Resource Package template generates ROS 2 packages focused on configuration files, URDF descriptions, launch files, and other non-executable resources.

## Overview

This template creates resource-focused ROS 2 packages containing:
- **URDF Robot Descriptions**: XML files defining robot kinematics and properties
- **Launch Configurations**: Python launch files for starting multiple nodes
- **Mesh Assets**: 3D model files for visualization
- **Configuration Files**: YAML/JSON parameter files
- **Documentation**: README and configuration guides
- **RViz Configurations**: View configurations for robot visualization

## Generated Structure

```
your_package/
├── package.xml                    # ROS 2 package manifest
├── CMakeLists.txt                # Minimal CMake configuration
├── README.md                      # Package documentation
├── CONTRIBUTING.md               # Development guidelines
├── Agents.md                      # AI interaction guide
├── urdf/
│   ├── robot.urdf                # Robot description file
│   └── robot.xacro               # Xacro parameterized description
├── meshes/
│   └── robot/
│       ├── base_link.stl         # 3D mesh files
│       └── link1.stl
├── launch/
│   ├── display.launch.py         # RViz display launch
│   ├── gazebo.launch.py          # Gazebo simulation launch
│   └── robot.launch.py           # Complete robot bringup
├── config/
│   ├── controllers.yaml          # Controller configurations
│   ├── joint_limits.yaml         # Joint limit definitions
│   └── rviz_config.rviz          # RViz visualization config
├── resource/
│   └── your_package              # Ament index resource
└── docs/
    └── setup.md                  # Setup and usage instructions
```

## Key Features

### URDF Robot Description
Complete robot description with kinematics:
```xml
<?xml version="1.0"?>
<robot name="your_robot">

  <!-- Links define the rigid bodies -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Joints define the connections between links -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1.0"/>
  </joint>

  <!-- Additional links and joints -->
  <link name="link1">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>

</robot>
```

### Xacro Parameterized Descriptions
Modular robot descriptions with parameters:
```xml
<?xml version="1.0"?>
<robot name="your_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Property definitions -->
  <xacro:property name="robot_name" value="my_robot"/>
  <xacro:property name="base_link_length" value="0.5"/>
  <xacro:property name="joint_effort" value="100"/>
  <xacro:property name="joint_velocity" value="1.0"/>

  <!-- Macro definitions -->
  <xacro:macro name="joint_with_limits" params="name type parent child xyz rpy axis">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <limit effort="${joint_effort}" velocity="${joint_velocity}" lower="-3.14159" upper="3.14159"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_link_length} ${base_link_length} 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Use the macro -->
  <xacro:joint_with_limits name="joint1" type="revolute"
                          parent="base_link" child="link1"
                          xyz="0 0 0.1" rpy="0 0 0" axis="0 0 1"/>

  <!-- Include other files -->
  <xacro:include filename="$(find your_package)/urdf/common_links.urdf.xacro"/>

</robot>
```

### Launch Files
Python launch configurations for complex setups:
```python
import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='your_robot',
        description='Name of the robot'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': launch.substitutions.Command([
                'xacro ', launch.substitutions.FindPackageShare('your_package'),
                '/urdf/robot.urdf.xacro robot_name:=', LaunchConfiguration('robot_name')
            ]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', launch.substitutions.PathJoinSubstitution([
            launch.substitutions.FindPackageShare('your_package'),
            'config', 'rviz_config.rviz'
        ])]
    )

    return LaunchDescription([
        use_sim_time,
        robot_name,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])
```

### RViz Configuration
Visualization setup for robot monitoring:
```yaml
# RViz configuration for robot visualization
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Splitter Ratio: 0.5
  - Class: rviz_common/Views
    Name: Views
    Property Tree Widget:
      Splitter Ratio: 0.5

Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Value: true
      Robot Description: robot_description
      TF Prefix: ""
    - Class: rviz_default_plugins/TF
      Name: TF
      Value: true
      Show Names: true
      Show Axes: true
      Show Arrows: true
      Marker Scale: 1.0
    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Value: true
      Topic: /scan
      Size (m): 0.1
      Color: 255; 0; 0
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
      Cell Size: 1.0
      Color: 160; 160; 164
  Global Options:
    Background Color: 48; 48; 48
    Frame Rate: 30
    Fixed Frame: base_link
  Tools:
    - Class: rviz_default_plugins/Interact
      Value: true
    - Class: rviz_default_plugins/MoveCamera
      Value: true
    - Class: rviz_default_plugins/Select
      Value: true
  Views:
    - Class: ""
      Name: Current View
      Type: rviz_default_plugins/Orbit
      Value: true
      Distance: 5.0
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.0
      Pitch: 0.0
      Yaw: 0.0
```

## Usage Examples

### Basic Robot Description
```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Wheel links -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Joints -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="-0.1 0.1 0" rpy="1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="-0.1 -0.1 0" rpy="1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

</robot>
```

### Controller Configuration
YAML configuration for robot controllers:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["wheel_left_joint"]
    right_wheel_names: ["wheel_right_joint"]

    wheel_separation: 0.2
    wheel_radius: 0.05

    use_stamped_vel: false
    publish_limited_velocity: true
    velocity_rolling_window_size: 10
    publish_rate: 50.0
    open_loop: true
    enable_odom_tf: true

    cmd_vel_timeout: 0.5
    publish_tf: true

    odom_frame_id: "odom"
    base_frame_id: "base_link"
    pose_covariance_diagonal : [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]

    # Limits
    linear.x.has_velocity_limits: true
    linear.x.has_acceleration_limits: true
    linear.x.has_jerk_limits: false
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -1.0
    linear.x.max_acceleration: 2.0
    linear.x.min_acceleration: -2.0

    angular.z.has_velocity_limits: true
    angular.z.has_acceleration_limits: true
    angular.z.has_jerk_limits: false
    angular.z.max_velocity: 3.0
    angular.z.min_velocity: -3.0
    angular.z.max_acceleration: 6.0
    angular.z.min_acceleration: -6.0
```

### Gazebo Simulation Launch
Launch file for Gazebo simulation:
```python
import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    world = DeclareLaunchArgument(
        'world',
        default_value='worlds/empty.world',
        description='Gazebo world file'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': launch.substitutions.Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('your_package'),
                    'urdf',
                    'robot.urdf.xacro'
                ])
            ])
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'my_robot'
        ]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('your_package'),
                'config',
                'controllers.yaml'
            ])
        ]
    )

    # Load controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        world,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        load_joint_state_controller,
        load_diff_drive_controller
    ])
```

## Configuration Options

### Robot Description Features
- **Links**: Define rigid bodies with visual, collision, and inertial properties
- **Joints**: Specify connections between links (revolute, prismatic, fixed, etc.)
- **Materials**: Define colors and textures for visualization
- **Transmissions**: Configure actuator interfaces
- **Gazebo Extensions**: Simulation-specific properties

### Launch Configuration Options
- **Composable Nodes**: Load components into existing processes
- **Lifecycle Management**: Control node startup sequences
- **Parameter Files**: Load YAML configuration files
- **Event Handlers**: Respond to process events
- **Conditional Execution**: Enable/disable components based on arguments

### Asset Organization
- **Meshes**: STL, OBJ, DAE files for 3D visualization
- **Textures**: Image files for material properties
- **CAD Files**: Source files for mesh generation
- **Documentation**: Setup guides and configuration references

## Building and Installation

### Package Installation
```bash
# Build the package
colcon build --packages-select your_package

# Source the workspace
source install/setup.bash

# Verify installation
ros2 pkg list | grep your_package
```

### Testing URDF
```bash
# Check URDF syntax
check_urdf urdf/robot.urdf

# View in RViz
ros2 launch your_package display.launch.py

# Validate Xacro
xacro urdf/robot.urdf.xacro
```

### Launch File Testing
```bash
# Test launch file syntax
ros2 launch your_package robot.launch.py --show-args

# Dry run (don't actually start processes)
ros2 launch your_package robot.launch.py --non-interactive
```

## Best Practices

### URDF Design
- Use consistent naming conventions
- Include collision geometries for navigation
- Define appropriate joint limits
- Use Xacro for complex robots
- Include inertial properties for simulation

### Launch File Organization
- Use descriptive argument names
- Include default values for all arguments
- Group related nodes logically
- Use conditional includes for optional components
- Document launch arguments

### File Organization
- Keep related files together
- Use descriptive directory names
- Include README files for complex configurations
- Version control large assets appropriately

## Integration with Other Packages

### Using Resource Packages
```python
# In another package's launch file
from launch_ros.substitutions import FindPackageShare

# Include robot description
robot_description_launch = IncludeLaunchDescription(
    PathJoinSubstitution([
        FindPackageShare('robot_description_package'),
        'launch',
        'description.launch.py'
    ])
)
```

### Parameter Files
```python
# Load parameters from resource package
config_file = PathJoinSubstitution([
    FindPackageShare('robot_config_package'),
    'config',
    'robot_config.yaml'
])

node = Node(
    package='robot_control',
    executable='robot_controller',
    parameters=[config_file]
)
```

## Troubleshooting

### Common Issues

#### URDF Parsing Errors
**Symptoms:** `check_urdf` reports syntax errors
**Solution:** Validate XML syntax and check for missing required attributes

#### Launch File Failures
**Symptoms:** Launch fails with import errors
**Solution:** Check Python path and ensure all dependencies are installed

#### RViz Display Issues
**Symptoms:** Robot not showing in RViz
**Solution:** Verify TF tree and robot_description parameter

#### Gazebo Spawn Failures
**Symptoms:** Robot not appearing in Gazebo
**Solution:** Check model format and spawn arguments

### Debug Tips
- Use `ros2 launch --debug` for detailed launch output
- Enable Gazebo verbose logging: `gzserver --verbose`
- Use `ros2 doctor` to check system health
- Validate URDF with `urdf_to_graphiz` for visualization

## Advanced Usage

### Modular Robot Descriptions
```xml
<!-- common_links.urdf.xacro -->
<xacro:macro name="wheel_link" params="name radius length color">
  <link name="${name}">
    <visual>
      <geometry>
        <cylinder radius="${radius}" length="${length}"/>
      </geometry>
      <material name="${color}_material">
        <color rgba="${color}"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${radius}" length="${length}"/>
      </geometry>
    </collision>
  </link>
</xacro:macro>
```

### Dynamic Launch Configurations
```python
def generate_launch_description():
    # Dynamic node creation based on configuration
    nodes = []

    # Load configuration
    config = load_yaml_config('robot_config.yaml')

    for sensor in config['sensors']:
        node = Node(
            package=sensor['package'],
            executable=sensor['executable'],
            name=sensor['name'],
            parameters=[sensor['params']]
        )
        nodes.append(node)

    return LaunchDescription(nodes)
```

### Multi-Robot Scenarios
```python
def generate_launch_description():
    robots = ['robot1', 'robot2', 'robot3']

    nodes = []
    for robot in robots:
        # Robot state publisher for each robot
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'rsp_{robot}',
            namespace=robot,
            parameters=[{
                'robot_description': get_robot_description(robot),
                'frame_prefix': f'{robot}/'
            }]
        )
        nodes.append(rsp)

    return LaunchDescription(nodes)
```

## Migration from ROS 1

Key differences when migrating resource packages:
- Update URDF syntax for ROS 2 conventions
- Convert launch files from XML to Python
- Update parameter file formats
- Use new RViz configuration format
- Update Gazebo integration methods
- Use ament instead of catkin

## Contributing

To improve the resource template:
1. Add more URDF examples for different robot types
2. Include additional launch file patterns
3. Add more RViz configuration examples
4. Include simulation-specific configurations
5. Add validation tools and best practices

See the [contributing guide](../CONTRIBUTING.md) for details on modifying templates.