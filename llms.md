# ROS Template VSCode Extension
This extension implements a wizard which creates a package for the robot operating system (ROS) 2. The package is created based on a template and the user can select options to include or exclude certain files from the package. The extension uses the [Handlebars.js](https://handlebarsjs.com/) templating engine to generate the package files.

In the templates directory of the extension, there are several templates for different types of packages. The templates are organized into subdirectories based on the type of package. Each template has a `manifest.yaml` file which describes the template and its options. All items except for the manifest file are copied from the template directory to the workspace, but can be customized based on the filemappings and options in the manifest file.

The manifest file is used to generate the package files. The manifest file contains the following fields:

```yaml
name: "template friendly name"
version: 0.0.0
repo: "http://github.com/..."
description: A templateMapping package
maintainers:
email:
license: "MIT"
options:
  - include_urdf: # The name of the option. This is used in the template to include or exclude files based on the option value.
    name: "Include URDF",   # The friendly name of the option
    description: "If this option is set to true, the corresponding file mapping will be included in the package.", # The description of the option for the user
    default: true, # The default value of the option. This is used if the user does not select a value for the option.
    type: "boolean" # The type of the option. Can be "boolean", "string", "number", "array", "object"

files:   # Specific files or directories are customized when the template directory is copied to the workspace.
  - "urdf/robot.urdf": # A specific file or directory relatve to template directory
      name: "urdf/{robot_name}.urdf", # The name of the file in the resulting package. The name can can include variables from the manifest file.
      condition: [include_urdf] # If specified, the file will only be included if the list of conditions are true.
```