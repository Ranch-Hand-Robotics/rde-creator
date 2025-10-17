# Contributing to ROS 2 Template Creator

Thank you for your interest in contributing to the ROS 2 Template Creator extension! This document provides guidelines for contributors.

## Development Setup

### Prerequisites
- Node.js 18+ and npm
- VS Code with extension development dependencies
- ROS 2 (any supported distribution)
- GitHub Copilot (recommended for testing AI features)

### Getting Started
1. Fork and clone the repository
2. Install dependencies: `npm install`
3. Open in VS Code
4. Run `npm run watch` to start development mode
5. Press `F5` to launch extension development host

## Architecture Overview

The extension uses a React-based webview for the UI and TypeScript for the backend. Key components:

- `src/extension.ts`: Main extension entry point
- `src/CreateNodePanel.ts`: Webview panel management
- `src/AIPackageGenerator.ts`: AI-powered package generation
- `src/webview/webview.tsx`: React UI components
- `src/templates/`: Template definitions and manifests

## Adding New Templates

### Template Structure

Templates are located in `src/templates/` and consist of:
- `manifest.yaml`: Template metadata and AI directives
- Template files (optional): Reference implementations

### Manifest Format

```yaml
name: "Template Display Name"
version: 0.0.0
description: "Detailed description of what this template creates"
license: "MIT"
icon: "🔧"  # Emoji icon for UI
short_description: "Brief tagline"
ai_directive: |
  HIGH PRIORITY REQUIREMENTS:
  - Specific instructions for AI generation
  - Code structure requirements
  - Best practices to follow
  - Language-specific guidelines

  # Include concrete code examples
  REQUIRED CODE STRUCTURE EXAMPLE:
  ```language
  // Example code showing expected structure
  ```

  # File organization
  Code Layout:
  package_name/
  ├── required_file.ext
  └── directory/
      └── another_file.ext
```

### Manifest Fields

| Field | Required | Description |
|-------|----------|-------------|
| `name` | Yes | Display name shown in UI |
| `version` | Yes | Template version |
| `description` | Yes | Detailed description |
| `license` | Yes | SPDX license identifier |
| `icon` | Yes | Emoji icon for visual identification |
| `short_description` | Yes | Brief tagline (under 50 chars) |
| `ai_directive` | Yes | Instructions for AI code generation |

### AI Directive Best Practices

1. **Be Specific**: Provide concrete code examples, not just descriptions
2. **Include Structure**: Show exact file layouts and class hierarchies
3. **Language Standards**: Specify version requirements (C++17+, Python 3.12+)
4. **ROS 2 Patterns**: Include proper node lifecycle, error handling, logging
5. **Cross-Platform**: Address Windows, Linux, macOS compatibility
6. **Testing**: Include test generation guidelines when applicable

### Template Validation

Before submitting:
1. Test template generation with various parameter combinations
2. Verify generated code compiles and runs
3. Check that tests pass (if included)
4. Validate manifest YAML syntax
5. Ensure AI directives produce consistent results

### Example: Adding a C++ Template

1. Create `src/templates/my_cpp_template/manifest.yaml`
2. Define comprehensive AI directive with code examples
3. Test generation with different configurations
4. Update documentation

## Code Style Guidelines

### TypeScript
- Use TypeScript strict mode
- Prefer `const` over `let`
- Use async/await over Promises
- Include JSDoc comments for public APIs

### React Components
- Use functional components with hooks
- Follow React best practices
- Include PropTypes for component props
- Use TypeScript interfaces for complex props

### Error Handling
- Use try/catch for async operations
- Provide meaningful error messages
- Log errors appropriately
- Handle edge cases gracefully

## Testing

### Unit Tests
- Located in `test/` directory
- Use Mocha test framework
- Run with `npm test`
- Cover critical paths and error conditions

### Integration Testing
- Test template generation end-to-end
- Verify generated packages build successfully
- Test AI generation with various prompts

### Manual Testing
- Test in extension development host
- Verify UI interactions
- Test with different ROS 2 distributions
- Validate generated code quality

## Pull Request Process

1. **Fork** the repository
2. **Create** a feature branch: `git checkout -b feature/your-feature`
3. **Make** your changes following the guidelines above
4. **Test** thoroughly (unit tests, integration tests, manual testing)
5. **Update** documentation as needed
6. **Commit** with clear, descriptive messages
7. **Push** to your fork
8. **Create** a Pull Request with:
   - Clear title and description
   - Reference to any related issues
   - Screenshots/videos for UI changes
   - Test results

## Documentation

### Updating Docs
- User-facing docs: `docs/` directory (MkDocs format)
- API docs: JSDoc comments in code
- Template docs: Update relevant template documentation

### Building Docs
```bash
# Install MkDocs if not already installed
pip install mkdocs mkdocs-material

# Serve docs locally
mkdocs serve

# Build docs
mkdocs build
```

## Release Process

### Version Bumping
- Update `package.json` version
- Update template versions in manifests
- Update changelog

### Publishing
- Run full test suite
- Build production extension: `npm run package`
- Publish to VS Code marketplace
- Create GitHub release

## Community

- **Issues**: Use GitHub issues for bugs and feature requests
- **Discussions**: Use GitHub discussions for questions and ideas
- **Code of Conduct**: Follow our community guidelines

## Recognition

Contributors will be recognized in:
- CHANGELOG.md for significant contributions
- GitHub repository contributors list
- Release notes

Thank you for contributing to the ROS 2 Template Creator!
