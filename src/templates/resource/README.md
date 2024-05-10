# {{package_name}}
{{package_description}}

## Developing
Clone this repository into an existing ROS workspace and build it using `colcon build` or use the Visual Studio Code ROS Extension:
```bash
git clone {{repository_url}}
cd {{package_name}}
colcon build
```

## Running
Run the package using the Visual Studio Code ROS Extension or by sourcing the workspace and running the launch file:
```bash
source install/setup.bash
ros2 launch {{package_name}} {{package_name}}.launch.py
```