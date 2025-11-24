// Copyright (c) Ranch Hand Robotics, LLC. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as yaml from 'yaml';
import { constructPlanPrompt, constructFileChunkPrompt, constructFollowupPromptInstructions } from './prompts';

export class AIPackageGenerator {
  private outputChannel: vscode.OutputChannel;
  private webview: vscode.Webview | undefined;
  private _isCancelled: boolean = false;
  
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
   * Cancel the current generation operation
   */
  public cancel(): void {
    this._isCancelled = true;
    this.outputChannel.appendLine('Generation cancellation requested');
  }

  /**
   * Check if generation has been cancelled
   */
  private checkCancellation(): void {
    if (this._isCancelled) {
      throw new Error('Generation cancelled by user');
    }
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
    selectedModelId?: string,
    selectedTestModelId?: string
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
      this.checkCancellation();

      const manifestContent = await fs.readFile(path.join(templatePath, 'manifest.yaml'), 'utf-8');
      const manifest = yaml.parse(manifestContent);
      this.sendProgress('Package manifest parsed');
      this.checkCancellation();

      // Stage 1: Request a generation plan (list files + chunking)
      const variablesObj = Object.fromEntries(variables);
      const planPrompt = constructPlanPrompt(templateContent, manifest, variablesObj, naturalLanguageDescription, this.maxResponseSize);
      const followupInstr = constructFollowupPromptInstructions(this.maxResponseSize);
      this.sendProgress('Sending plan request to AI model...');
      const planResponse = await model.sendRequest([
        vscode.LanguageModelChatMessage.User(planPrompt + '\n' + followupInstr)
      ], { justification: 'Request generation plan (file list + chunking)' });
      const planObj = await this.processAIResponse(planResponse);
      this.sendProgress('Plan received from AI');
      this.checkCancellation();

      // Normalize plan into a map of file -> estimated_chunks
      const filesList: Array<{path: string; estimate_bytes?: number}> = [];
      const chunksMap: Record<string, number> = {};

      if (Array.isArray(planObj.files)) {
        for (const f of planObj.files) {
          if (typeof f === 'string') { filesList.push({ path: f }); }
          else if (f && f.path) { filesList.push({ path: f.path, estimate_bytes: f.estimate_bytes }); }
        }
      }
      
      // Send file list to webview
      if (filesList.length > 0) {
        const processedFilesList = filesList.map(f => this.processVariables(f.path, variables));
        if (this.webview) {
          this.webview.postMessage({ 
            command: 'aiPlan', 
            files: processedFilesList 
          });
        }
        this.sendProgress(`Generation plan includes ${filesList.length} files`);
      }
      if (planObj.chunks && typeof planObj.chunks === 'object') {
        for (const [k, v] of Object.entries(planObj.chunks)) {
          const n = Number(v) || 0;
          if (n > 0) { chunksMap[k] = Math.min(10, Math.max(1, n)); }
        }
      }

      // Fallback: if no files listed, try manifest file mapping or throw
      if (filesList.length === 0) {
        // try common package files
        filesList.push({ path: 'package.xml' });
      }

      // Decide chunk counts heuristically when not provided
      for (const f of filesList) {
        if (!chunksMap[f.path]) {
          const estimate = f.estimate_bytes || 2000;
          const perChunk = Math.max(1000, Math.floor(this.maxResponseSize * 0.6));
          const needed = Math.min(10, Math.max(1, Math.ceil(estimate / perChunk)));
          chunksMap[f.path] = needed;
        }
      }

      // Stage 2: Request each file chunk-by-chunk and stream to disk
      // Generate files in parallel (max 3 at a time to avoid overwhelming the API)
      const MAX_PARALLEL_FILES = 3;
      
      const generateFile = async (f: {path: string; estimate_bytes?: number}) => {
        if (!model) {
          throw new Error('AI model not available');
        }
        const total = chunksMap[f.path] || 1;
        
        // Prepare file path and ensure directory exists
        const processedPath = this.processVariables(f.path, variables);
        const fullPath = path.join(targetDirectory, processedPath);
        await fs.mkdir(path.dirname(fullPath), { recursive: true });
        
        // Open file for writing (truncate if exists)
        let fileHandle: fs.FileHandle | undefined;
        try {
          fileHandle = await fs.open(fullPath, 'w');
          
          // Stream chunks directly to disk (must be sequential per file)
          for (let i = 1; i <= total; i++) {
            this.checkCancellation();
            const chunkPrompt = constructFileChunkPrompt(templateContent, manifest, variablesObj, naturalLanguageDescription, f.path, i, total);
            this.sendProgress(`Requesting chunk ${i}/${total} for ${f.path}`);
            const chunkResp = await model.sendRequest([
              vscode.LanguageModelChatMessage.User(chunkPrompt + '\n' + followupInstr)
            ], { justification: `Request file chunk ${i}/${total} for ${f.path}` });
            const chunkObj = await this.processAIResponse(chunkResp);
            if (!chunkObj || chunkObj.file !== f.path) {
              throw new Error(`Unexpected chunk response for ${f.path}`);
            }
            
            // Unescape and process chunk content
            const contentEscaped = String(chunkObj.content || '');
            const unescaped = contentEscaped.replace(/\\n/g, '\n');
            const processedContent = this.processVariables(unescaped, variables);
            
            // Validate variable replacement
            if (processedContent.includes('{{')) {
              this.outputChannel.appendLine(`WARNING: Unreplaced variables found in chunk ${i} of ${processedPath}`);
            }
            
            // Write chunk directly to file
            await fileHandle.write(processedContent, null, 'utf-8');
            this.sendProgress(`Written chunk ${i}/${total} to ${processedPath}`);
          }
          
          this.sendProgress(`Completed file: ${processedPath}`);
        } finally {
          if (fileHandle) {
            await fileHandle.close();
          }
        }
      };
      
      // Process files in parallel batches
      for (let i = 0; i < filesList.length; i += MAX_PARALLEL_FILES) {
        this.checkCancellation();
        const batch = filesList.slice(i, i + MAX_PARALLEL_FILES);
        await Promise.all(batch.map(f => generateFile(f)));
      }

      this.sendProgress('ROS 2 package files created successfully');

      // Stage 3: Generate tests if requested
      if (testDescription && testDescription.trim() && selectedTestModelId) {
        this.checkCancellation();
        this.sendProgress('Starting test generation with separate AI model...');
        
        // Select test model
        let testModel: vscode.LanguageModelChat | undefined;
        if (selectedTestModelId && selectedTestModelId !== 'auto') {
          const allModels = await vscode.lm.selectChatModels({});
          testModel = allModels.find(m => `${m.vendor}-${m.family}-${allModels.indexOf(m)}` === selectedTestModelId);
        }
        
        if (!testModel) {
          this.sendProgress('Warning: Test model not found, skipping test generation');
        } else {
          this.outputChannel.appendLine(`Using test model: ${testModel.name} (${testModel.vendor}, ${testModel.family})`);
          
          // Request test plan
          const testPlanPrompt = this.constructTestPlanPrompt(templateContent, manifest, variablesObj, naturalLanguageDescription, testDescription);
          this.sendProgress('Requesting test generation plan...');
          const testPlanResponse = await testModel.sendRequest([
            vscode.LanguageModelChatMessage.User(testPlanPrompt + '\n' + followupInstr)
          ], { justification: 'Request test generation plan' });
          const testPlanObj = await this.processAIResponse(testPlanResponse);
          this.sendProgress('Test plan received from AI');
          
          // Normalize test plan
          const testFilesList: Array<{path: string; estimate_bytes?: number}> = [];
          const testChunksMap: Record<string, number> = {};
          
          if (Array.isArray(testPlanObj.files)) {
            for (const f of testPlanObj.files) {
              if (typeof f === 'string') { testFilesList.push({ path: f }); }
              else if (f && f.path) { testFilesList.push({ path: f.path, estimate_bytes: f.estimate_bytes }); }
            }
          }
          
          // Send test file list to webview
          if (testFilesList.length > 0) {
            const processedTestFilesList = testFilesList.map(f => this.processVariables(f.path, variables));
            if (this.webview) {
              this.webview.postMessage({ 
                command: 'aiTestPlan', 
                files: processedTestFilesList 
              });
            }
            this.sendProgress(`Test generation plan includes ${testFilesList.length} files`);
          }
          if (testPlanObj.chunks && typeof testPlanObj.chunks === 'object') {
            for (const [k, v] of Object.entries(testPlanObj.chunks)) {
              const n = Number(v) || 0;
              if (n > 0) { testChunksMap[k] = Math.min(10, Math.max(1, n)); }
            }
          }
          
          // Decide chunk counts for test files
          for (const f of testFilesList) {
            if (!testChunksMap[f.path]) {
              const estimate = f.estimate_bytes || 2000;
              const perChunk = Math.max(1000, Math.floor(this.maxResponseSize * 0.6));
              const needed = Math.min(10, Math.max(1, Math.ceil(estimate / perChunk)));
              testChunksMap[f.path] = needed;
            }
          }
          
          // Generate test files in parallel
          const generateTestFile = async (f: {path: string; estimate_bytes?: number}) => {
            if (!testModel) {
              throw new Error('Test AI model not available');
            }
            const total = testChunksMap[f.path] || 1;
            const processedPath = this.processVariables(f.path, variables);
            const fullPath = path.join(targetDirectory, processedPath);
            await fs.mkdir(path.dirname(fullPath), { recursive: true });
            
            let fileHandle: fs.FileHandle | undefined;
            try {
              fileHandle = await fs.open(fullPath, 'w');
              
              for (let i = 1; i <= total; i++) {
                this.checkCancellation();
                const testChunkPrompt = this.constructTestFileChunkPrompt(templateContent, manifest, variablesObj, naturalLanguageDescription, testDescription, f.path, i, total);
                this.sendProgress(`Requesting test chunk ${i}/${total} for ${f.path}`);
                const testChunkResp = await testModel.sendRequest([
                  vscode.LanguageModelChatMessage.User(testChunkPrompt + '\n' + followupInstr)
                ], { justification: `Request test file chunk ${i}/${total} for ${f.path}` });
                const testChunkObj = await this.processAIResponse(testChunkResp);
                
                if (!testChunkObj || testChunkObj.file !== f.path) {
                  throw new Error(`Unexpected test chunk response for ${f.path}`);
                }
                
                const contentEscaped = String(testChunkObj.content || '');
                const unescaped = contentEscaped.replace(/\\n/g, '\n');
                const processedContent = this.processVariables(unescaped, variables);
                
                if (processedContent.includes('{{')) {
                  this.outputChannel.appendLine(`WARNING: Unreplaced variables found in test chunk ${i} of ${processedPath}`);
                }
                
                await fileHandle.write(processedContent, null, 'utf-8');
                this.sendProgress(`Written test chunk ${i}/${total} to ${processedPath}`);
              }
              
              this.sendProgress(`Completed test file: ${processedPath}`);
            } finally {
              if (fileHandle) {
                await fileHandle.close();
              }
            }
          };
          
          // Process test files in parallel batches
          for (let i = 0; i < testFilesList.length; i += MAX_PARALLEL_FILES) {
            const batch = testFilesList.slice(i, i + MAX_PARALLEL_FILES);
            await Promise.all(batch.map(f => generateTestFile(f)));
          }
          
          this.sendProgress('Test files generated successfully');
        }
      }

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

    // Clean up the response - remove markdown code blocks if present
    let cleanedResponse = fullResponse.trim();
    
    // Try to extract JSON from markdown code blocks first (most common case)
    const markdownJsonMatch = cleanedResponse.match(/```(?:json)?\s*\n?(\{[\s\S]*?\})\s*\n?```/);
    if (markdownJsonMatch) {
      cleanedResponse = markdownJsonMatch[1].trim();
    } else if (cleanedResponse.startsWith('```')) {
      // Handle cases where markdown wrapping exists but without json tag
      const codeBlockMatch = cleanedResponse.match(/```[^\n]*\n([\s\S]*?)```/);
      if (codeBlockMatch) {
        cleanedResponse = codeBlockMatch[1].trim();
      }
    }

    try {
      // Try to parse the cleaned response as JSON
      const parsed = JSON.parse(cleanedResponse);
      return parsed;
    } catch (error) {
      // Fallback: Try to extract JSON object from text (find first { to matching })
      const jsonMatch = cleanedResponse.match(/\{[\s\S]*\}/);
      if (jsonMatch) {
        try {
          return JSON.parse(jsonMatch[0]);
        } catch (regexError) {
          this.sendProgress(`Failed to parse regex-extracted JSON: ${regexError}`);
        }
      }

      // Fallback 2: Try to find JSON after common prefixes
      const prefixesToTry = ['Here is the ROS 2 package:', 'Here\'s the package:', 'Package structure:', 'Generated package:', 'Here is the file:', 'Here\'s the chunk:'];
      for (const prefix of prefixesToTry) {
        const prefixIndex = fullResponse.indexOf(prefix);
        if (prefixIndex !== -1) {
          const afterPrefix = fullResponse.substring(prefixIndex + prefix.length).trim();
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

      this.sendProgress(`Failed to parse AI response as JSON: ${error}`);
      this.sendProgress(`Response preview: ${fullResponse.substring(0, 200)}...`);
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

  private constructTestPlanPrompt(
    templateContent: string,
    manifest: any,
    variablesObj: Record<string, string>,
    packageDescription: string,
    testDescription: string
  ): string {
    const aiDirective = manifest.ai_directive || '';
    return `You are an expert ROS 2 test engineer. Produce a JSON plan for generating test files for a ROS 2 package.

IMPORTANT: Return ONLY raw JSON. Do NOT wrap in markdown code blocks. Do NOT use \`\`\`json formatting.
Your response must start with { and end with }.

Example output format:
{
  "files":[{"path":"test/test_node.cpp","estimate_bytes":500}],
  "chunks":{}
}

Package functionality:
${packageDescription}

Test requirements:
${testDescription}

Requirements:
- List all test files to generate under "files" as objects: {"path":"...","estimate_bytes":123}
- Provide a "chunks" map: file path -> number of chunks (<=10) to split large files
- Use ${this.maxResponseSize} as an upper bound for any single response size
- Include appropriate test frameworks (gtest for C++, unittest/pytest for Python, Jest for Node.js)
- Follow ROS 2 testing best practices
${aiDirective ? `
TEMPLATE AI DIRECTIVE:
${aiDirective}
` : ''}
User variables: ${JSON.stringify(variablesObj)}
`;
  }

  private constructTestFileChunkPrompt(
    templateContent: string,
    manifest: any,
    variablesObj: Record<string, string>,
    packageDescription: string,
    testDescription: string,
    filePath: string,
    chunkIndex: number,
    totalChunks: number
  ): string {
    const aiDirective = manifest.ai_directive || '';
    return `You are an expert ROS 2 test engineer. Return JSON for test chunk ${chunkIndex}/${totalChunks} of file: ${filePath}

IMPORTANT: Return ONLY raw JSON. Do NOT wrap in markdown code blocks. Do NOT use \`\`\`json formatting.
Your response must start with { and end with }.

Example output format:
{
  "file":"${filePath}",
  "chunk_index":${chunkIndex},
  "total_chunks":${totalChunks},
  "content":"test file content here with \\n for newlines"
}

Package functionality:
${packageDescription}

Test requirements:
${testDescription}

Requirements:
- Content must be a JSON string with special characters escaped
- Use \\n for newlines, escape quotes as \\" and backslashes as \\\\
- chunk_index is 1-based
- Concatenation of all chunks in order must equal the full test file content
- Generate ONLY this chunk (${chunkIndex}/${totalChunks}), not the entire file
- Follow ROS 2 testing best practices
- Include proper test fixtures, setup/teardown, and assertions
- Test edge cases and error conditions
${aiDirective ? `
TEMPLATE AI DIRECTIVE:
${aiDirective}
` : ''}
User variables: ${JSON.stringify(variablesObj)}
`;
  }
}