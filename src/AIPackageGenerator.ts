import * as vscode from 'vscode';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as yaml from 'yaml';
import { ProcessCreateNode } from './ProcessCreateNode';

export class AIPackageGenerator {
  private outputChannel: vscode.OutputChannel;

  constructor(outputChannel: vscode.OutputChannel) {
    this.outputChannel = outputChannel;
  }

  /**
   * Generate a ROS package using AI based on template, parameters, and natural language description
   */
  public async generatePackageWithAI(
    templatePath: string,
    variables: Map<string, string>,
    naturalLanguageDescription: string,
    targetDirectory: string
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
      const manifestContent = await fs.readFile(path.join(templatePath, 'manifest.yaml'), 'utf-8');
      const manifest = yaml.parse(manifestContent);

      // Construct the AI prompt
      const prompt = this.constructAIPrompt(templateContent, manifest, variables, naturalLanguageDescription);

      // Send request to language model
      const messages = [
        vscode.LanguageModelChatMessage.User(prompt)
      ];

      const response = await model.sendRequest(messages, {
        justification: 'Generating ROS 2 package code based on template and user requirements'
      });

      // Process the AI response
      const generatedContent = await this.processAIResponse(response);

      // Generate the package using the AI response
      await this.createPackageFromAIResponse(generatedContent, targetDirectory, variables);

      this.outputChannel.appendLine('AI-powered ROS package generation completed successfully');

    } catch (error) {
      this.outputChannel.appendLine(`AI generation failed: ${error}`);
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
    naturalLanguageDescription: string
  ): string {
    const variablesObj = Object.fromEntries(variables);

    return `
You are an expert ROS 2 developer tasked with generating a complete ROS 2 package based on a template and user requirements.

## Template Information
The following template files are available:
${templateContent}

## Package Manifest
${JSON.stringify(manifest, null, 2)}

## User Parameters
${JSON.stringify(variablesObj, null, 2)}

## User Requirements
${naturalLanguageDescription}

## Instructions
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

    for await (const part of response.text) {
      fullResponse += part;
    }

    this.outputChannel.appendLine(`AI Response received (${fullResponse.length} characters)`);

    try {
      // Try to parse as JSON
      const parsed = JSON.parse(fullResponse);
      return parsed;
    } catch (error) {
      this.outputChannel.appendLine(`Failed to parse AI response as JSON: ${error}`);
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
        this.outputChannel.appendLine(`Created directory: ${dirPath}`);
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
        this.outputChannel.appendLine(`Created file: ${fullPath}`);
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
}