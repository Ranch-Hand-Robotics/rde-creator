# ROS 2 Template Creator Extension

An AI-powered Visual Studio Code extension for creating ROS 2 packages from intelligent templates. This extension leverages GitHub Copilot and Language Models to generate complete, functional ROS 2 packages in C++, Python, and Node.js.

![Creator](https://raw.githubusercontent.com/Ranch-Hand-Robotics/rde-creator/refs/heads/main/media/screenshot.png)

## Features

### 🤖 AI-Powered Package Generation
- **Natural Language Descriptions**: Describe your ROS 2 node in plain English and let AI generate the complete implementation
- **Template-Based Creation**: Choose from pre-built templates for common ROS 2 patterns
- **Multi-Language Support**: Generate packages in C++, Python, and Node.js
- **Intelligent Code Completion**: Templates include GitHub Copilot prompts for enhanced development

### 📦 Available Templates

#### C++ ROS 2 Node ⚡
High-performance, real-time ROS 2 composable nodes with:
- Modern C++17+ standards
- Proper visibility control for Windows/Linux compatibility
- Component-based architecture with `rclcpp_components`
- Optional lifecycle management
- Comprehensive error handling and logging

#### Python ROS 2 Node 🐍
Fast prototyping with rich libraries:
- Modern Python 3.12+ syntax
- Async/await patterns for concurrent operations
- Comprehensive docstrings and type hints
- Built-in test generation with pytest

#### Node.js ROS 2 Node 🌐
Web integration and async I/O:
- Full rclnodejs integration
- Promise-based asynchronous operations
- Web-friendly development patterns
- REST API integration capabilities

#### Resource Package 📁
Configuration and asset management:
- URDF files for robot descriptions
- Launch configurations
- Mesh files and assets
- Configuration files and parameters

## Quick Start

### Installation
1. Install the extension from the VS Code marketplace
2. Ensure you have ROS 2 installed and sourced
3. Install GitHub Copilot Chat extension for AI-powered generation

### Creating Your First Package

1. **Open Command Palette** (`Ctrl+Shift+P` / `Cmd+Shift+P`)
2. **Run Command**: "ROS 2: Create Package"
3. **Choose Template**: Select from C++, Python, Node.js, or Resource templates
4. **Configure Options**: Set package metadata and features
5. **AI Generation** (Optional): Describe your node functionality in natural language
6. **Generate**: The extension creates a complete, buildable ROS 2 package

### Alternative: Context Menu Creation
- **Right-click** in the Explorer panel on any folder
- **Select**: "Create ROS 2 Package Here"
- **Follow the same configuration steps**

## Template Details

### C++ Template Features
- **Composable Nodes**: Full `rclcpp_components` support
- **Visibility Control**: Cross-platform symbol visibility management
- **Lifecycle Support**: Optional `rclcpp_lifecycle` integration
- **Modern CMake**: Automatic ROS distribution detection
- **Testing**: Integrated gtest framework with proper discovery

### Python Template Features
- **Async Support**: Native asyncio integration
- **Type Hints**: Full type annotation support
- **Testing**: pytest integration with ROS 2 fixtures
- **Documentation**: Auto-generated docstrings

### Node.js Template Features
- **Promise-Based**: Modern async/await patterns
- **Web Integration**: HTTP server capabilities
- **Testing**: Jest integration with ROS 2 mocks

## Development Workflow

1. **Generate Package**: Use the extension to create your initial package structure
2. **Customize**: Modify generated code to fit your specific requirements
3. **Build & Test**: Use standard ROS 2 tools (`colcon build`, `colcon test`)
4. **Iterate**: Leverage GitHub Copilot for code enhancements and debugging

## Requirements

- **VS Code**: Latest version recommended (1.101.0+)
- **ROS 2**: Humble, Iron, Jazzy, or Kilted
- **GitHub Copilot**: For AI-powered generation (optional but recommended)
- **Node.js**: For the extension itself (comes with VS Code)
- **GitHub Copilot CLI** (optional): For enhanced background agent functionality
  - Install with: `npm install -g @github/copilot`
  - The extension will automatically fall back to VS Code's Language Model API if the CLI is not available

## AI Generation Modes

This extension supports two AI generation modes:

### 1. GitHub Copilot SDK (Recommended)
The extension uses the new GitHub Copilot SDK for background agent functionality when available. This provides:
- More robust session management
- Better error handling
- Improved streaming response processing
- Official SDK support from GitHub

To use this mode:
1. Install the Copilot CLI globally: `npm install -g @github/copilot`
2. The extension will automatically detect and use the SDK

### 2. VS Code Language Model API (Fallback)
If the Copilot SDK is unavailable, the extension automatically falls back to using VS Code's built-in Language Model API. This mode:
- Works with any installed language model provider
- Requires GitHub Copilot Chat extension
- Provides full functionality with slightly different architecture

You can configure which mode to use in the extension settings:
- `rosPackageCreator.useCopilotSDK`: Enable/disable Copilot SDK usage
- `rosPackageCreator.copilotCLIPath`: Custom path to Copilot CLI (leave empty for default)

## Troubleshooting

### Common Issues

**"Language Model API not available"**
- Ensure GitHub Copilot Chat extension is installed and active
- Check that you have a valid GitHub Copilot subscription
- If using Copilot SDK mode, ensure the CLI is installed

**"Failed to initialize Copilot SDK"**
- Install the GitHub Copilot CLI: `npm install -g @github/copilot`
- Verify the CLI is accessible in your PATH
- Check extension logs in the "RDE Creator" output channel
- The extension will automatically fall back to vscode.lm API

**"Template generation failed"**
- Verify ROS 2 environment is properly sourced
- Check that target directory is writable
- Ensure template manifests are valid
- Check for error details in the "RDE Creator" output channel

**"Build failures after generation"**
- Verify all ROS 2 dependencies are installed
- Check that generated CMakeLists.txt matches your ROS distribution
- Ensure visibility control headers are properly configured

## License

MIT License - see [LICENSE](LICENSE) for details