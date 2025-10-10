import * as vscode from 'vscode';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as yaml from 'yaml';

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

      // Try to get user's preferred model from VS Code configuration
      const config = vscode.workspace.getConfiguration('rosPackageCreator');
      const preferredFamily = config.get<string>('preferredModelFamily', 'gpt-5-mini');
      
      this.outputChannel.appendLine(`User preferred model family: ${preferredFamily}`);

      // Select models based on user preference, defaulting to Copilot vendor
      let models: vscode.LanguageModelChat[] = [];
      let selectionMethod = 'user configuration';

      // Try to get models with preferred family from Copilot
      models = await vscode.lm.selectChatModels({
        vendor: 'copilot',
        family: preferredFamily
      });

      if (models.length === 0) {
        // Fallback: try any Copilot model
        models = await vscode.lm.selectChatModels({
          vendor: 'copilot'
        });
        selectionMethod = `fallback (requested ${preferredFamily} not available)`;
      } else {
        selectionMethod = `user preference (${preferredFamily})`;
      }

      // Continue with existing fallback logic if still no models
      if (models.length === 0) {
        models = await vscode.lm.selectChatModels({
          family: preferredFamily
        });
        selectionMethod = `any vendor (${preferredFamily})`;
      }

      if (models.length === 0) {
        models = await vscode.lm.selectChatModels({});
        selectionMethod = 'any available model';
      }

      if (models.length === 0) {
        throw new Error('No language models available');
      }

      const model = models[0];
      this.outputChannel.appendLine(`Available models: ${models.map(m => `${m.name} (${m.family})`).join(', ')}`);
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
      throw error; // Re-throw instead of falling back to template processing
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
          // Skip manifest.yaml files - they're metadata, not template content
          if (file === 'manifest.yaml') {
            continue;
          }

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

${aiDirective ? `## TEMPLATE-SPECIFIC AI DIRECTIVE (HIGH PRIORITY - FOLLOW EXACTLY)
${aiDirective}

CRITICAL: Follow the directory structure and file organization specified in the AI directive exactly.
CRITICAL: Only include test files if explicitly requested by the user in the test requirements section.
CRITICAL: Do not include any files not mentioned in the AI directive unless specifically requested.
CRITICAL: Respond with VALID JSON only. Your response must be parseable as JSON and contain only the package structure.


` : ''}## Template Information
The following template files are available for reference:
${templateContent}

## Package Manifest (Metadata - Do not include manifest.yaml in generated package)
${JSON.stringify(manifest, null, 2)}

## User Parameters (Replace {{variable}} placeholders with these values)
${JSON.stringify(variablesObj, null, 2)}

## User Requirements (CRITICAL - Implement exactly as specified)
${naturalLanguageDescription}

**IMPORTANT**: The user's natural language description above is the PRIMARY requirement. 
Analyze it carefully and implement EXACTLY what is requested. Do not add extra features or change the functionality unless explicitly asked.
If the description is unclear, make reasonable assumptions but document them in comments.

## Test Requirements
${testDescription && testDescription.trim() ? `
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

` : 'IMPORTANT: No tests were requested by the user. Do NOT include any test files or test directories in the generated package.'}

## Instructions
1. **CRITICAL**: Follow the exact directory structure and file layout specified in the AI directive
2. **CRITICAL**: Create files as needed to support the request
3. **CRITICAL**: Do not include manifest.yaml or any metadata files in the generated package
4. Analyze the user's natural language description and adapt the package accordingly
5. Use the provided parameters to customize the package (replace ALL {{variable}} placeholders)
6. Generate a complete, functional ROS 2 package that meets the user's requirements
7. Ensure all generated files follow ROS 2 best practices and conventions
8. Include appropriate dependencies, launch files, and configuration as needed
9. Make the package production-ready with proper error handling and documentation
10. Use modern ROS 2 patterns (rclpy for Python, rclcpp for C++, rclnodejs for Node.js, rclrust for Rust)
11. Include proper QoS settings where appropriate
12. Add comprehensive logging and error handling
13. Follow ROS 2 naming conventions and message standards
14. Include launch files with proper parameter handling
15. ${testDescription && testDescription.trim() ? 'Generate comprehensive test suites based on test requirements' : '**DO NOT generate any test files** - no tests were requested'}
16. Ensure test files follow language-specific testing conventions and ROS 2 patterns
17. **IMPORTANT:** Generate a comprehensive CONTRIBUTING.md file that documents:
    - Complete package architecture with Mermaid diagrams showing:
      * Node architecture and communication patterns
      * Data flow between components
      * Service/action interfaces
      * Parameter dependencies
      * Launch file structure
    - Detailed explanation of each node's purpose and responsibilities
    - Communication patterns (publishers, subscribers, services, actions)
    - Parameter documentation with defaults and valid ranges
    - Build and installation instructions
    - Testing strategy and how to run tests
    - Development guidelines and code organization
    - Dependencies and their purposes
    - Troubleshooting common issues
    - The original user requirements that generated this package
    - AI generation metadata (date, model, user prompt summary)

## Important ROS 2 Requirements
- Include spin() or equivalent for event processing
- Use appropriate message types from standard ROS 2 packages
- Include proper exception handling
- Add docstrings and comments for maintainability
- Do not include any placeholder text like "TODO" or "FIXME".
- If you are confused about any requirement, make a reasonable assumption and proceed, but add a comment explaining your confusion and decisions used to move forward.

Generate the ROS 2 package now:
## Output Format (JSON only)
{
  "directories": ["dir1", "dir2"],
  "files": {
    "path/to/file.py": "file content here",
    "package.xml": "<?xml version='1.0'?>..."
  }
}
`;
  }

  private async processAIResponse(response: vscode.LanguageModelChatResponse): Promise<any> {
    let fullResponse = '';
    let lastLoggedLength = 0;
    const logInterval = 5000; // Log every 1000 characters

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
      // Try to parse as JSON directly first
      const parsed = JSON.parse(fullResponse);
      return parsed;
    } catch (error) {
      this.sendProgress(`Failed to parse AI response :\n ${fullResponse}\n`);
      this.sendProgress(`Failed to parse AI response as JSON: ${error}`);

      // Fallback 1: Try to extract JSON from markdown code blocks
      const markdownJsonMatch = fullResponse.match(/```(?:json)?\s*\n?(\{[\s\S]*?\})\s*\n?```/);
      if (markdownJsonMatch) {
        try {
          return JSON.parse(markdownJsonMatch[1]);
        } catch (markdownError) {
          this.sendProgress(`Failed to parse extracted markdown JSON: ${markdownError}`);
        }
      }

      // Fallback 2: Try to extract JSON using the original regex (finds first { to last })
      const jsonMatch = fullResponse.match(/\{[\s\S]*\}/);
      if (jsonMatch) {
        try {
          return JSON.parse(jsonMatch[0]);
        } catch (regexError) {
          this.sendProgress(`Failed to parse regex-extracted JSON: ${regexError}`);
        }
      }

      // Fallback 3: Try to find JSON after common prefixes
      const prefixesToTry = ['Here is the ROS 2 package:', 'Here\'s the package:', 'Package structure:', 'Generated package:'];
      for (const prefix of prefixesToTry) {
        const prefixIndex = fullResponse.indexOf(prefix);
        if (prefixIndex !== -1) {
          const afterPrefix = fullResponse.substring(prefixIndex + prefix.length).trim();
          try {
            const parsed = JSON.parse(afterPrefix);
            return parsed;
          } catch (prefixError) {
            // Try to extract JSON from the remaining text
            const jsonFromRest = afterPrefix.match(/\{[\s\S]*\}/);
            if (jsonFromRest) {
              try {
                return JSON.parse(jsonFromRest[0]);
              } catch (restError) {
                // Continue to next prefix
              }
            }
          }
        }
      }

      throw new Error(`AI response is not valid JSON. Response starts with: "${fullResponse.substring(0, 100)}..."`);
    }
  }

  private async createPackageFromAIResponse(
    aiResponse: any,
    targetDirectory: string,
    variables: Map<string, string>
  ): Promise<void> {
    // Validate that we have the required structure
    if (!aiResponse.files && !aiResponse.directories) {
      throw new Error('AI response must contain either "files" or "directories" properties');
    }

    // Log variables for debugging
    this.outputChannel.appendLine(`Processing variables: ${JSON.stringify(Object.fromEntries(variables))}`);

    // Create directories first
    if (aiResponse.directories) {
      for (const dir of aiResponse.directories) {
        const processedDir = this.processVariables(dir, variables);
        const dirPath = path.join(targetDirectory, processedDir);
        await fs.mkdir(dirPath, { recursive: true });

        this.sendProgress(`Created directory: ${processedDir}`);
      }
    }

    // Create files
    if (aiResponse.files) {
      for (const [filePath, content] of Object.entries(aiResponse.files)) {
        const processedPath = this.processVariables(filePath, variables);
        const fullPath = path.join(targetDirectory, processedPath);
        const processedContent = this.processVariables(content as string, variables);

        // Validate variable replacement
        if (processedPath.includes('{{') || processedContent.includes('{{')) {
          this.outputChannel.appendLine(`WARNING: Unreplaced variables found in ${processedPath}`);
        }

        // Ensure directory exists
        await fs.mkdir(path.dirname(fullPath), { recursive: true });

        await fs.writeFile(fullPath, processedContent, 'utf-8');
        this.sendProgress(`Created file: ${processedPath}`);
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
}