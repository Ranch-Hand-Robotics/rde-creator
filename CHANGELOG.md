# Change Log

All notable changes to the "robotics-templates" extension will be documented in this file.

Check [Keep a Changelog](http://keepachangelog.com/) for recommendations on how to structure this file.

## [Unreleased]

### Added
- **MicroROS2 Package Template**: New template for Raspberry Pi Pico (RP2040/RP2350) firmware generation
  - Pico SDK + CMake build system
  - Docker-based GitHub template fetching (micro_ros_raspberrypi_pico_sdk)
  - PowerShell scripts for Windows (build_firmware.ps1, fetch_pico_template.ps1)
  - Bash scripts for Linux/macOS
  - ROS distribution selector (kilted, jazzy, iron, humble, rolling)
  - USB control for UF2 bootloader upload (libusb)
  - Auto-build and upload functionality
- **Next Steps Documentation**: Post-generation guidance displayed on completion page
  - Template-specific instructions in manifest.yaml
  - Markdown rendering with proper formatting
  - Coverage for configuration, dependencies, testing, and deployment
- **Multi-File Batching**: Optimized AI generation for small files
  - Batches up to 5 files in single API call
  - 15% size threshold for intelligent batching
  - Reduces API calls and improves generation speed
- **Resource Package Next Steps**: Comprehensive guidance for URDF, meshes, launch files, and Gazebo worlds

### Fixed
- **File Completion Animation**: Files now properly marked as completed when AI generation finishes
- **Progress Indicators**: Multi-file batch generation now shows proper spinner animations for each file
- **Markdown Rendering**: Next steps documentation renders correctly with headers, code blocks, lists, and formatting
- **State Management**: Generation completion properly updates all file status indicators
- **Macro Generation Issues**: Removed problematic RCCHECK/RCSOFTCHECK macros in favor of direct error checking

### Changed
- Reduced multi-file batching from 30% to 15% threshold for better reliability
- Limited batch size to maximum 5 files to prevent AI content dropping
- Improved progress messages for better user feedback during generation
- Enhanced webview state persistence across navigation

### Technical Improvements
- Added `convertMarkdownToHtml()` function for proper markdown-to-HTML conversion
- Implemented chunk index validation to detect AI duplication
- Increased token limits to 1MB for large package generation
- Added proper message handlers for multi-file batch progress
- Improved error handling and fallback mechanisms in AI generation