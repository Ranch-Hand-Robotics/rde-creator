# ROS 2 Template Extension - AI Assistant Guidelines

## Key Instructions !required
Do not use powershell, run commands in cmd or bash depending on the OS. 

When generating code, ensure that:
- All code is compatible with the latest stable versions of Node.js and TypeScript.
- The React code uses functional components and hooks. 
- The code adheres to best practices for VS Code extension development, including proper use of the VS Code API.
- The Handlebars templates are correctly formatted and include necessary conditionals based on the manifest options.
- The webpack configuration supports both development and production builds, with appropriate optimizations for each.
- The extension follows the structure and conventions outlined in the provided documentation, including file naming, directory structure, and coding standards.

## Project Overview
This is a VS Code extension that generates ROS 2 packages from Handlebars-based templates. It features a React webview UI for template selection and configuration, with a dual-webpack build system.

## Architecture Patterns

### Core Components
- **`src/extension.ts`**: Main extension entry point, registers commands and manages output channel
- **`src/CreateNodePanel.ts`**: Webview panel management, handles VS Code webview lifecycle and messaging
- **`src/ProcessCreateNode.ts`**: Template processing engine using Handlebars for file generation
- **`src/webview/webview.tsx`**: React-based UI for template selection and configuration
- **`src/utils.ts`**: Utility functions for manifest loading, file operations, and VS Code integration

### Template System
Templates are defined in `src/templates/` with YAML manifests:
```yaml
name: "Template Name"
options:
  - variable: "include_feature"
    name: "Include Feature"
    type: "boolean"
file_mapping:
  - "source/path": "destination/{{variable}}_file.ext"
    condition: include_feature
```

### Build System
Dual webpack configuration in `webpack.config.js`:
- **Extension config**: `target: 'node'`, outputs `dist/extension.js`
- **Webview config**: `target: 'web'`, outputs `dist/webview.js` with React bundle

## Key Conventions

### Communication Protocol
Extension ↔ Webview communication uses structured messages:
```typescript
// Extension to webview
{ command: "setManifests", manifests: "json_string" }

// Webview to extension
{
  command: "createPackage",
  type: "template_name",
  variables: { package_name: "value", ... }
}
```

### File Naming
- Use `fileNameFromVariable()` for converting user input to filesystem-safe names
- Pattern: `variableValue.replace(/ /g, "_").toLowerCase()`

### Template Processing
- Templates use Handlebars with conditional file inclusion
- Variables are passed as Map<string, string> to template processor
- File mapping supports conditional copying based on user selections

## Development Workflow

### Building
```bash
npm run compile    # Build both extension and webview
npm run watch      # Watch mode for development
npm run package    # Production build with minification
```

### Testing
```bash
npm run pretest    # Build and lint before tests
npm run test       # Run VS Code extension tests
```

### Adding Templates
1. Create template directory in `src/templates/`
2. Add `manifest.yaml` with options and file mappings
3. Template files use Handlebars syntax: `{{variable}}`
4. Update webpack copy patterns if needed

## VS Code Integration

### Webview Setup
- Uses `acquireVsCodeApi()` for extension communication
- Content Security Policy requires nonce for scripts
- Local resources served via `webview.asWebviewUri()`

### Extension Commands
- `robotics-templates.create`: Opens template selection webview
- Available in explorer context menu for folders

## Common Patterns

### Error Handling
```typescript
extension.outputChannel.appendLine(`Error: ${error}`);
vscode.window.showErrorMessage(`Error: ${error}`);
```

### File Operations
```typescript
// Use vscode.Uri for path operations
const templateUri = vscode.Uri.joinPath(extensionUri, 'dist', 'templates');

// Use fs/promises for async file operations
const content = await fs.readFile(filePath, 'utf-8');
```

### State Management
React webview uses standard hooks:
- `useState` for form state and selections
- `useEffect` for message handling from extension
- `postMessage` for sending data back to extension

## Template Development

### Manifest Structure
```yaml
options:
  - variable: "option_name"
    name: "Display Name"
    description: "User-friendly description"
    type: "boolean" | "string"
    default: "default_value"
    condition: "other_option"  # Only show if other option is true
```

### File Mapping
```yaml
file_mapping:
  - "source/file.ext": "dest/{{variable}}.ext"
    condition: option_name
  - "entire_directory/":
    condition: include_directory
```

## Debugging Tips

### Webview Issues
- Check browser console in webview developer tools
- Verify CSP nonces match between HTML generation and script tags
- Ensure `process.env` is properly polyfilled for React

### Template Issues
- Check manifest YAML syntax
- Verify Handlebars template variables are properly escaped
- Test conditional logic in file mappings

### Extension Issues
- Use `extension.outputChannel` for logging (viewable in Output panel)
- Check VS Code developer console for extension errors
- Verify webpack externals configuration for vscode modules