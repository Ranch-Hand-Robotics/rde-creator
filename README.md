# Visual Studio Code ROS 2 Template extension
From the maintainer of the [Visual Studio Code ROS Extension](http://aka.ms/ros/vscode), this extension adds the following fuctions for creating, modifying and maintaining ROS 2 Nodes:

* Initialize a ROS 2 workspace on an open folder
  * `Optional` Set up Dev Containers
  * `Optional` initialize repos file
* Create a ROS 2 Node
  * Create C++, Python, Rust, C# based ROS 2 Node
  * Initialize from various Node templates including
    * Basic
      * `Optional` With Publisher
      * `Optional` With Subscriber
      * `Optional` With Action Server
      * `Optional` With Service
    * I2C Node
    * GPIO Node
    * Joint State Node
    * `Optional` With Tests
    * `Optional` With Launch file
   
## ROS Node Template Framework
ROS 2 Node templates are based on [Handlebars.js](https://handlebarsjs.com/).

## Github Copilot Commenting
ROS 2 Templates provided by this extension include [Github Copilot](https://github.com/features/copilot/) 'Prompt comments', which assist Copilot in generating appropriate code for the project. 


## Templates

### Manifest Format

```yaml
name: "template friendly name"
version: 0.0.0
repo: "http://github.com/..."
description: A templateMapping package
maintainers:
email:
license: "MIT"
options:
  - include_urdf:
    name: "Include URDF",
    description: "Include a URDF file in the package",
    type: "boolean"

  - include_launch: 
    name: "Include Launch",
    description: "Include a launch file in the package",
    type: "boolean"

  - include_meshes:
    name: "Include Meshes",
    description: "Include Meshes in the package",
    type: "boolean"

  - robot_name:
    name: "Robot Name",
    description: "Name of the Robot",
    type: "string",
    default: "robot",
    condition: include_urdf
file_mapping:
  - "urdf/robot.urdf":
      name: "urdf/{robot_name}}.urdf",
      condition: include_urdf
```