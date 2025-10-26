// Copyright (c) Ranch Hand Robotics, LLC. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as yaml from 'yaml';
import { constructAIPrompt } from './prompts';

export class AIPackageGenerator {
  private outputChannel: vscode.OutputChannel;
  private webview: vscode.Webview | undefined;
  
  // Maximum AI response size in characters (configurable)
  // Default: 50KB - increase if you need larger/more complex packages
  // Decrease if you're hitting memory issues
  private readonly maxResponseSize: number;

  constructor(
    outputChannel: vscode.OutputChannel, 
    webview?: vscode.Webview,
    maxResponseSize: number = 50000
  ) {
    this.outputChannel = outputChannel;
    this.webview = webview;
    this.maxResponseSize = maxResponseSize;
  }

  /**
   * Generate a ROS package using AI based on template, parameters, and natural language description
   */
  public async generatePackageWithAI(
    templatePath: string,
    variables: Map<string, string>,
    naturalLanguageDescription: string,
    targetDirectory: string,
    testDescription?: string,
    selectedModelId?: string
  ): Promise<void> {
    try {
      // Check if language model access is available
      if (!vscode.lm) {
        throw new Error('Language Model API is not available in this VS Code version');
      }

      let model: vscode.LanguageModelChat | undefined;
      let selectionMethod = 'default';

      // If a specific model was selected by the user, try to find it
      if (selectedModelId && selectedModelId !== 'auto') {
        const allModels = await vscode.lm.selectChatModels({});
        model = allModels.find(m => `${m.vendor}-${m.family}-${allModels.indexOf(m)}` === selectedModelId);
        if (model) {
          selectionMethod = `user selected (${model.name})`;
          this.outputChannel.appendLine(`Using user-selected model: ${model.name} (${model.vendor}, ${model.family})`);
        }
      }

      // If no specific model was selected or found, use the configuration-based selection
      if (!model) {
          vscode.window.showErrorMessage("Unfortunately the selected language model is not available. Please select a different model and try again.");
          return;
      }

      // Read template files and manifest
      const templateContent = await this.readTemplateContent(templatePath);
      this.sendProgress('Template files loaded and analyzed');

      const manifestContent = await fs.readFile(path.join(templatePath, 'manifest.yaml'), 'utf-8');
      const manifest = yaml.parse(manifestContent);
      this.sendProgress('Package manifest parsed');

      // Construct the AI prompt with the configured max response size
      const prompt = constructAIPrompt(
        templateContent, 
        manifest, 
        variables, 
        naturalLanguageDescription, 
        testDescription,
        this.maxResponseSize
      );
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

  private async processAIResponse(response: vscode.LanguageModelChatResponse): Promise<any> {
    let fullResponse = '';
    let lastLoggedLength = 0;
    const logInterval = 5000; // Log every 5000 characters

    for await (const part of response.text) {
      fullResponse += part;
      
      // Check if response is getting too large
      if (fullResponse.length > this.maxResponseSize) {
        throw new Error(`Response too long (${fullResponse.length} characters). The AI generated more than ${this.maxResponseSize} characters. Try simplifying your request or increase the maxResponseSize limit.`);
      }
      
      // Only log progress every 5000 characters to reduce spam
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

        // Check if this path was already created as a directory
        try {
          const stats = await fs.stat(fullPath);
          if (stats.isDirectory()) {
            this.outputChannel.appendLine(`ERROR: Cannot create file at ${processedPath} - path already exists as directory`);
            continue; // Skip this file
          }
        } catch (error) {
          // Path doesn't exist, which is expected - continue with file creation
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