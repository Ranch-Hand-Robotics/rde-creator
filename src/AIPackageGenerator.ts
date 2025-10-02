import * as vscode from 'vscode';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as yaml from 'yaml';
import { ProcessCreateNode } from './ProcessCreateNode';

export class AIPackageGenerator {
  private outputChannel: vscode.OutputChannel;
  private webview: vscode.Webview | undefined;

  constructor(outputChannel: vscode.OutputChannel, webview?: vscode.Webview) {
    this.outputChannel = outputChannel;
    this.webview = webview;
  }

  /**
   * Generate a ROS package using AI based on template, parameters, and natural language description
   */
  public async generatePackageWithAI(
    templatePath: string,
    variables: Map<string, string>,
    naturalLanguageDescription: string,
    targetDirectory: string,
    testDescription?: string
  ): Promise<void> {
    try {
      // Check if language model access is available
      if (!vscode.lm) {
        throw new Error('Language Model API is not available in this VS Code version');
      }

      // Try to select the user's preferred model first (no specific family constraint)
      let models = await vscode.lm.selectChatModels({
        vendor: 'copilot'
      });
      let selectionMethod = 'user preferred (copilot vendor)';

      // If no models found with just vendor, try with the specific family
      if (models.length === 0) {
        models = await vscode.lm.selectChatModels({
          vendor: 'copilot',
          family: 'gpt-4o'
        });
        selectionMethod = 'fallback to gpt-4o family';
      }

      // If still no models, try without vendor constraint to get any available model
      if (models.length === 0) {
        models = await vscode.lm.selectChatModels({});
        selectionMethod = 'any available model';
      }

      if (models.length === 0) {
        throw new Error('No language models available');
      }

      const model = models[0];
      this.outputChannel.appendLine(`Using language model: ${model.name} (${model.vendor}, ${model.family}) - Selected via: ${selectionMethod}`);

      // Read template files and manifest
      const templateContent = await this.readTemplateContent(templatePath);
      this.sendProgress('Template files loaded and analyzed');

      const manifestContent = await fs.readFile(path.join(templatePath, 'manifest.yaml'), 'utf-8');
      const manifest = yaml.parse(manifestContent);
      this.sendProgress('Package manifest parsed');

      // Construct the AI prompt
      const prompt = this.constructAIPrompt(templateContent, manifest, variables, naturalLanguageDescription, testDescription);
      this.sendProgress('AI prompt constructed with template and user requirements');

      // Send request to language model
      const messages = [
        vscode.LanguageModelChatMessage.User(prompt)
      ];

      this.sendProgress('Sending request to AI language model...');
      const response = await model.sendRequest(messages, {
        justification: 'Generating ROS 2 package code based on template and user requirements'
      });
      this.sendProgress('AI model response received, processing...');

      // Process the AI response
      const generatedContent = await this.processAIResponse(response);
      this.sendProgress('AI response processed and parsed');

      // Generate the package using the AI response
      await this.createPackageFromAIResponse(generatedContent, targetDirectory, variables);
      this.sendProgress('ROS 2 package files created successfully');

      // Generate CONTRIBUTING.md for AI-generated package
      await this.generateAIContributingMd(
        targetDirectory,
        variables,
        naturalLanguageDescription,
        testDescription || '',
        prompt
      );
      this.sendProgress('CONTRIBUTING.md documentation generated');

      this.outputChannel.appendLine('AI-powered ROS package generation completed successfully');
      // Send completion message to webview
      if (this.webview) {
        this.webview.postMessage({ command: 'aiComplete' });
      }

    } catch (error) {
      this.outputChannel.appendLine(`AI generation failed: ${error}`);
      // Send completion message to webview
      if (this.webview) {
        this.webview.postMessage({ command: 'aiComplete' });
      }
      // Fallback to traditional template processing
      this.outputChannel.appendLine('Falling back to traditional template processing...');
      const processor = new ProcessCreateNode(templatePath);
      await processor.createResourcePackage(targetDirectory, variables);
    }
  }

  private async readTemplateContent(templatePath: string): Promise<string> {
    const templateFiles: string[] = [];
    const queue: string[] = [templatePath];

    while (queue.length > 0) {
      const currentPath = queue.shift()!;
      const files = await fs.readdir(currentPath);

      for (const file of files) {
        const filePath = path.join(currentPath, file);
        const stats = await fs.stat(filePath);

        if (stats.isDirectory()) {
          queue.push(filePath);
        } else {
          // Read template file content
          const content = await fs.readFile(filePath, 'utf-8');
          const relativePath = path.relative(templatePath, filePath);
          templateFiles.push(`=== ${relativePath} ===\n${content}\n`);
        }
      }
    }

    return templateFiles.join('\n');
  }

  private constructAIPrompt(
    templateContent: string,
    manifest: any,
    variables: Map<string, string>,
    naturalLanguageDescription: string,
    testDescription?: string
  ): string {
    const variablesObj = Object.fromEntries(variables);

    // Extract AI directive from manifest if present
    const aiDirective = manifest.ai_directive || '';

    return `
You are an expert ROS 2 developer tasked with generating a complete ROS 2 package based on a template and user requirements.

${aiDirective ? `## TEMPLATE-SPECIFIC AI DIRECTIVE (HIGH PRIORITY)
${aiDirective}

` : ''}## Template Information
The following template files are available:
${templateContent}

## Package Manifest
${JSON.stringify(manifest, null, 2)}

## User Parameters
${JSON.stringify(variablesObj, null, 2)}

## User Requirements
${naturalLanguageDescription}

${testDescription && testDescription.trim() ? `## Test Requirements
The user has specified the following test case requirements:
${testDescription}

Please generate comprehensive test cases that:
- Test the functionality described above
- Cover edge cases and error scenarios mentioned
- Follow ROS 2 testing best practices
- Include unit tests, integration tests, and any specific test types mentioned
- Use appropriate test frameworks (gtest for C++, unittest/pytest for Python, Jest for Node.js)
- Include proper setup and teardown procedures
- Test ROS 2 specific functionality (publishers, subscribers, services, parameters)
- Validate message flows and service calls
- Include performance and reliability tests where mentioned

` : ''}## Instructions
1. Analyze the template structure and understand the ROS 2 package layout
2. Consider the user's natural language description and adapt the package accordingly
3. Use the provided parameters to customize the package (replace {{variable}} placeholders)
4. Generate a complete, functional ROS 2 package that meets the user's requirements
5. Ensure all generated files follow ROS 2 best practices and conventions
6. Include appropriate dependencies, launch files, and configuration as needed
7. Make the package production-ready with proper error handling and documentation
8. Use modern ROS 2 patterns (rclpy for Python, rclcpp for C++)
9. Include proper QoS settings where appropriate
10. Add comprehensive logging and error handling
11. Follow ROS 2 naming conventions and message standards
12. Include launch files with proper parameter handling
13. Generate comprehensive test suites based on test requirements (if provided)
14. Ensure test files follow language-specific testing conventions and ROS 2 patterns

## Important ROS 2 Requirements
- Use the correct import statements for ROS 2 (rclpy, rclcpp)
- Implement proper node lifecycle management
- Include spin() or equivalent for event processing
- Use appropriate message types from standard ROS 2 packages
- Follow ROS 2 parameter conventions
- Include proper exception handling
- Add docstrings and comments for maintainability

## Output Format
Provide the complete package as a JSON object with the following structure:
{
  "files": {
    "path/to/file1": "file content",
    "path/to/file2": "file content",
    ...
  },
  "directories": ["path/to/dir1", "path/to/dir2", ...]
}

Generate the ROS 2 package now:
`;
  }

  private async processAIResponse(response: vscode.LanguageModelChatResponse): Promise<any> {
    let fullResponse = '';
    let lastLoggedLength = 0;
    const logInterval = 1000; // Log every 1000 characters

    for await (const part of response.text) {
      fullResponse += part;
      
      // Only log progress every 1000 characters to reduce spam
      if (fullResponse.length - lastLoggedLength >= logInterval) {
        this.sendProgress(`Receiving AI response... (${fullResponse.length} characters so far)`);
        lastLoggedLength = fullResponse.length;
      }
    }

    this.sendProgress(`AI Response received (${fullResponse.length} characters)`);

    try {
      // Try to parse as JSON
      const parsed = JSON.parse(fullResponse);
      return parsed;
    } catch (error) {
      this.sendProgress(`Failed to parse AI response as JSON: ${error}`);

      // Fallback: try to extract JSON from the response
      const jsonMatch = fullResponse.match(/\{[\s\S]*\}/);
      if (jsonMatch) {
        return JSON.parse(jsonMatch[0]);
      }
      throw new Error('AI response is not valid JSON');
    }
  }

  private async createPackageFromAIResponse(
    aiResponse: any,
    targetDirectory: string,
    variables: Map<string, string>
  ): Promise<void> {
    // Create directories first
    if (aiResponse.directories) {
      for (const dir of aiResponse.directories) {
        const dirPath = path.join(targetDirectory, this.processVariables(dir, variables));
        await fs.mkdir(dirPath, { recursive: true });

        this.sendProgress(`Created directory: ${dirPath}`);
      }
    }

    // Create files
    if (aiResponse.files) {
      for (const [filePath, content] of Object.entries(aiResponse.files)) {
        const processedPath = this.processVariables(filePath, variables);
        const fullPath = path.join(targetDirectory, processedPath);
        const processedContent = this.processVariables(content as string, variables);

        // Ensure directory exists
        await fs.mkdir(path.dirname(fullPath), { recursive: true });

        await fs.writeFile(fullPath, processedContent, 'utf-8');
        this.sendProgress(`Created file: ${fullPath}`);
      }
    }
  }

  private processVariables(text: string, variables: Map<string, string>): string {
    let result = text;
    for (const [key, value] of variables) {
      result = result.replace(new RegExp(`{{${key}}}`, 'g'), value);
    }
    return result;
  }

  private sendProgress(message: string) {
    this.outputChannel.appendLine(`AI Progress: ${message}`);
    if (this.webview) {
      this.webview.postMessage({ command: 'aiProgress', text: message });
    }
  }

  /**
   * Generates CONTRIBUTING.md for AI-generated packages
   */
  private async generateAIContributingMd(
    targetDirectory: string,
    variables: Map<string, string>,
    naturalLanguageDescription: string,
    testDescription: string,
    aiPrompt: string
  ): Promise<void> {
    try {
      // Create a mock manifest for AI-generated packages
      const aiManifest = {
        name: "AI Generated Package",
        ai_directive: this.extractAIDirectiveFromPrompt(aiPrompt)
      };

      // Use ProcessCreateNode's CONTRIBUTING.md generation logic
      const processCreateNode = new ProcessCreateNode('');
      
      await processCreateNode.generateContributingMd(
        targetDirectory,
        variables,
        aiManifest,
        true, // isAiGenerated
        aiPrompt,
        naturalLanguageDescription,
        testDescription
      );
    } catch (error) {
      this.outputChannel.appendLine(`Error generating AI CONTRIBUTING.md: ${error}`);
    }
  }

  /**
   * Direct CONTRIBUTING.md generation for AI packages
   */
  private async generateContributingMdDirect(
    targetDirectory: string,
    variables: Map<string, string>,
    naturalLanguageDescription: string,
    testDescription: string,
    aiPrompt: string
  ): Promise<void> {
    const packageName = variables.get('package_name') || 'ai_generated_package';
    const packageDescription = variables.get('package_description') || 'AI generated ROS 2 package';
    const maintainer = variables.get('package_maintainer') || 'AI Generated';
    const version = variables.get('package_version') || '0.0.0';
    const license = variables.get('package_license') || 'MIT';

    const contributingContent = `# Contributing to ${packageName}

This document provides information about the structure and architecture of this AI-generated ROS 2 package.

## Package Overview

**Package Name:** ${packageName}  
**Description:** ${packageDescription}  
**Version:** ${version}  
**License:** ${license}  
**Maintainer:** ${maintainer}

## AI Generation Details

This package was generated using AI-powered code generation with the following specifications:

### User Requirements
\`\`\`
${naturalLanguageDescription}
\`\`\`

${testDescription ? `### Test Requirements
\`\`\`
${testDescription}
\`\`\`

` : ''}### AI Prompt Used
<details>
<summary>Full AI Prompt (Click to expand)</summary>

\`\`\`
${aiPrompt}
\`\`\`
</details>

## Package Architecture

### System Overview

\`\`\`mermaid
graph TB
    subgraph "${packageName} Package"
        NODE[AI Generated Node]
        ${variables.get('include_publisher') === 'true' ? 'NODE --> |publishes| TOPIC[Topic]' : ''}
        ${variables.get('include_subscriber') === 'true' ? 'TOPIC_IN[Input Topic] --> |subscribes| NODE' : ''}
        ${variables.get('include_service') === 'true' ? 'CLIENT --> |request| NODE\n        NODE --> |response| CLIENT' : ''}
    end
\`\`\`

### Communication Patterns

The package implements the following ROS 2 communication patterns based on the AI analysis:

${variables.get('include_publisher') === 'true' ? `#### Publisher Pattern
- Publishes messages on configured topics
- Uses appropriate QoS settings for reliability

` : ''}${variables.get('include_subscriber') === 'true' ? `#### Subscriber Pattern  
- Subscribes to input topics
- Processes messages according to requirements

` : ''}${variables.get('include_service') === 'true' ? `#### Service Pattern
- Provides service endpoints for external requests
- Implements proper error handling and responses

` : ''}## Development Guidelines

### Building the Package

\`\`\`bash
cd ~/ros2_ws
colcon build --packages-select ${packageName}
source install/setup.bash
\`\`\`

### Running the Package

\`\`\`bash
# Check the generated launch files for specific run instructions
ros2 launch ${packageName} <launch_file>.launch.py

# Or run nodes directly
ros2 run ${packageName} <node_name>
\`\`\`

### Testing

\`\`\`bash
# Run package tests
colcon test --packages-select ${packageName}
colcon test-result --verbose
\`\`\`

## AI Generation Metadata

- **Generated On:** ${new Date().toISOString().split('T')[0]}
- **AI Model:** GitHub Copilot
- **Generation Type:** Template-guided AI generation
- **Template Base:** ROS 2 best practices

## Contributing

This package was generated by AI based on user requirements. To contribute:

1. Understand the original requirements (see above)
2. Follow ROS 2 best practices and conventions
3. Add appropriate tests for new functionality
4. Update this documentation for significant changes

---

*This CONTRIBUTING.md was automatically generated using AI-powered code generation on ${new Date().toISOString().split('T')[0]}.*
`;

    const contributingPath = path.join(targetDirectory, 'CONTRIBUTING.md');
    await fs.writeFile(contributingPath, contributingContent, 'utf-8');
    this.outputChannel.appendLine(`Generated AI CONTRIBUTING.md at ${contributingPath}`);
  }

  /**
   * Extracts AI directive from the full prompt
   */
  private extractAIDirectiveFromPrompt(prompt: string): string {
    // Try to extract the template-specific directive section
    const directiveMatch = prompt.match(/## TEMPLATE-SPECIFIC AI DIRECTIVE.*?\n(.*?)\n\n/s);
    if (directiveMatch) {
      return directiveMatch[1].trim();
    }
    
    // Fallback: return first part of prompt
    const lines = prompt.split('\n');
    return lines.slice(0, Math.min(10, lines.length)).join('\n');
  }
}