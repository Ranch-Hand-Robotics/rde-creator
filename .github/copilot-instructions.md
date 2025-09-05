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
This is a VS Code extension that generates ROS 2 packages from Handlebars-based templates. It features a React webview UI for template selection and configuration, AI-powered package generation via VS Code's Language Model API, and a dual-webpack build system. The extension supports both traditional template-based generation and AI-powered generation with natural language descriptions.

## Architecture Patterns

### Core Components
- **`src/extension.ts`**: Main extension entry point, registers commands and manages output channel
- **`src/CreateNodePanel.ts`**: Webview panel management, handles VS Code webview lifecycle and messaging with folder context support
- **`src/ProcessCreateNode.ts`**: Template processing engine using Handlebars for file generation with conditional file mapping
- **`src/AIPackageGenerator.ts`**: AI-powered package generation using VS Code Language Model API with fallback to traditional processing
- **`src/webview/webview.tsx`**: React-based UI with three-page workflow (package selection, configuration, AI progress)
- **`src/utils.ts`**: Utility functions for manifest loading, file operations, and VS Code integration with async filesystem support

### Template System
Templates are defined in `src/templates/` with YAML manifests that support AI directives:
```yaml
name: "Template Name"
description: "Template description"
ai_directive: |
  Template-specific AI instructions for code generation
  with specific requirements and guidelines
options:
  - variable: "include_feature"
    name: "Include Feature"
    type: "boolean"
    condition: "other_option"  # Conditional visibility
file_mapping:
  - "source/path": "destination/{{variable}}_file.ext"
    condition: include_feature
  - "entire_directory/":
      condition: include_directory
```

### AI Generation System
The extension supports AI-powered package generation with:
- **Language Model Integration**: Uses VS Code's Language Model API with fallback mechanism
- **Template-Aware Prompting**: AI receives complete template context and user requirements
- **Natural Language Processing**: Users can describe package functionality in plain English
- **Progressive Generation**: Real-time progress updates with step-by-step feedback
- **Fallback Strategy**: Automatic fallback to traditional template processing if AI fails

### Build System
Dual webpack configuration in `webpack.config.js`:
- **Extension config**: `target: 'node'`, outputs `dist/extension.js`
- **Webview config**: `target: 'web'`, outputs `dist/webview.js` with React bundle

## Key Conventions

### Communication Protocol
Extension ↔ Webview communication uses structured messages with enhanced AI support:
```typescript
// Extension to webview
{ command: "setManifests", manifests: "json_string" }
{ command: "aiProgress", text: "progress_message" }
{ command: "aiComplete" }

// Webview to extension
{
  command: "createPackage" | "createPackageWithAI",
  type: "template_name",
  variables: { package_name: "value", ... },
  naturalLanguageDescription?: "AI generation requirements"
}
```

### Folder Context Support
- Extension supports right-click creation from Explorer context menu
- Target folder URI is preserved through webview operations
- Packages are created in the selected folder or workspace root as fallback

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
- `robotics-templates.create`: Opens template selection webview with optional folder context
- Available in explorer context menu for folders with URI parameter support
- Command can be invoked from Command Palette or by right-clicking folders in Explorer

### Webview UI Architecture
The React webview implements a three-page workflow:
1. **Package Selection**: Choose template type and set metadata
2. **Configuration**: Select options, configure AI generation, set natural language description
3. **Progress/Generation**: Real-time AI generation progress with step tracking and log display

#### Navigation Features
- Browser-style back navigation with state preservation
- Header bars with contextual back buttons
- Proper history management for webview navigation
- Full-width responsive layouts optimized for VS Code webview

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
React webview uses comprehensive state management:
- `useState` for form state, page navigation, AI generation status, and progress tracking
- `useEffect` for message handling, manifest loading timeouts, and browser navigation hooks
- `postMessage` for bidirectional communication with extension
- Loading states and timeout handling for manifest operations
- Progress tracking for AI generation with real-time updates

## Template Development

### Manifest Structure with AI Support
```yaml
name: "Template Display Name"
description: "Template description for AI context"
ai_directive: |
  Specific instructions for AI generation
  Include ROS 2 patterns, conventions, and requirements
  Specify language preferences and architectural patterns
options:
  - variable: "option_name"
    name: "Display Name"
    description: "User-friendly description"
    type: "boolean" | "string"
    default: "default_value"
    condition: "other_option"  # Only show if other option is true
```

### File Mapping with Conditional Logic
```yaml
file_mapping:
  - "source/file.ext": "dest/{{variable}}.ext"
    condition: option_name
  - "entire_directory/":
      condition: include_directory
      files:
        - "nested/file.ext": "output/{{variable}}.ext"
```

### AI Directive Guidelines
When writing AI directives for templates:
- Specify ROS 2 conventions and best practices
- Include language preferences (Python/C++)
- Define architecture patterns and code structure
- Mention required dependencies and imports
- Specify error handling and logging requirements
- Include performance and scalability considerations

## AI Generation Workflow

### Language Model Integration
```typescript
// Model selection with fallback strategy
let models = await vscode.lm.selectChatModels({ vendor: 'copilot' });
if (models.length === 0) {
  models = await vscode.lm.selectChatModels({ vendor: 'copilot', family: 'gpt-4o' });
}
if (models.length === 0) {
  models = await vscode.lm.selectChatModels({});
}
```

### AI Prompt Construction
The AI prompt includes:
- Complete template file content and structure
- Parsed manifest with options and AI directives
- User-provided variables and parameters
- Natural language description of requirements
- ROS 2 best practices and conventions
- Output format specification (JSON structure)

### Response Processing
- Stream processing for real-time progress updates
- JSON parsing with fallback extraction methods
- File and directory creation from AI response
- Variable substitution in generated content
- Error handling with fallback to traditional processing

### Progress Reporting
```typescript
private sendProgress(message: string) {
  this.outputChannel.appendLine(`AI Progress: ${message}`);
  if (this.webview) {
    this.webview.postMessage({ command: 'aiProgress', text: message });
  }
}
```

## Debugging Tips

### AI Generation Issues
- Check VS Code Language Model API availability (`vscode.lm`)
- Verify GitHub Copilot Chat extension is installed and active
- Monitor AI prompt construction and template content reading
- Check AI response parsing and JSON extraction
- Verify fallback mechanism activation

### Webview Navigation Issues
- Check browser history state management
- Verify back button event handling
- Test page state preservation during navigation
- Monitor message passing between pages

### Manifest Loading Issues
- Check timeout handling for manifest operations
- Verify YAML parsing and file system access
- Test retry mechanism for failed manifest loading
- Monitor workspace URI resolution and template paths

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
- Monitor folder context handling and URI parameter passing
- Test command registration and context menu integration

## Recent Architecture Improvements

### Asynchronous AI Generation
- Non-blocking AI operations to prevent UI freezing
- Progressive generation with real-time feedback
- Proper error handling and fallback mechanisms
- Resource cleanup and webview lifecycle management

### Enhanced User Experience
- Three-page workflow with intuitive navigation
- Full-width responsive layouts optimized for VS Code
- Browser-style navigation with back button support
- Real-time progress tracking with visual indicators
- Contextual help and error messaging

### Robust Manifest Management
- Timeout-based manifest loading with retry capability
- Async filesystem operations with proper error handling
- Map-based manifest storage for efficient access
- Graceful degradation when templates are unavailable

### Folder Context Integration
- Right-click folder creation with preserved context
- URI-based target folder specification
- Workspace fallback for command palette invocation
- Proper path resolution across different VS Code scenarios

## Performance Considerations

### Template Processing
- Iterative file copying with BFS traversal
- Efficient conditional file mapping evaluation
- Memory-conscious template content reading
- Optimized variable substitution patterns

### AI Integration
- Streaming response processing for large generations
- Efficient prompt construction with minimal duplication
- Resource cleanup after AI operations
- Network timeout handling and retry logic

### React Performance
- Functional components with optimized re-rendering
- Efficient state management with minimal updates
- Proper event listener cleanup
- Optimized CSS with VS Code theme integration