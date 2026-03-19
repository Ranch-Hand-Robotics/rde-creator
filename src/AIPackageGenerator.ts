// Copyright (c) Ranch Hand Robotics, LLC. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import * as fs from 'fs/promises';
import * as path from 'path';
import * as yaml from 'yaml';
import { constructPlanPrompt, constructFileChunkPrompt, constructFollowupPromptInstructions } from './prompts';
import { CopilotSDKService } from './CopilotSDKService';

export class AIPackageGenerator {
  private outputChannel: vscode.OutputChannel;
  private webview: vscode.Webview | undefined;
  private _isCancelled: boolean = false;
  private sdkService: CopilotSDKService;
  
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
    this.sdkService = new CopilotSDKService(outputChannel);
  }

  /**
   * Cancel the current generation operation
   */
  public cancel(): void {
    this._isCancelled = true;
    this.outputChannel.appendLine('Generation cancellation requested');
    // Abort SDK session if active
    this.sdkService.abort().catch(err => {
      this.outputChannel.appendLine(`Error aborting SDK session: ${err}`);
    });
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
      // Initialize SDK - this will throw if SDK is not available
      await this.sdkService.initialize();
      this.sendProgress('Using GitHub Copilot SDK for package generation');
      this.outputChannel.appendLine('Using Copilot SDK for AI generation');

      // Create a session
      await this.sdkService.createSession(selectedModelId !== 'auto' ? selectedModelId : undefined);

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
      const planResponseText = await this.sendAIRequest(planPrompt + '\n' + followupInstr);
      const planObj = await this.processAIResponse(planResponseText);
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
      
      // Common file generation function
      const generateFileWithChunks = async (
        f: {path: string; estimate_bytes?: number},
        chunksMapping: Record<string, number>,
        promptConstructor: (filePath: string, chunkIndex: number, totalChunks: number) => string,
        messagePrefix: string
      ) => {
        const total = chunksMapping[f.path] || 1;
        
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
            const chunkPrompt = promptConstructor(f.path, i, total);
            this.sendProgress(`Requesting ${messagePrefix}chunk ${i}/${total} for ${f.path}`);
            const chunkRespText = await this.sendAIRequest(chunkPrompt + '\n' + followupInstr);
            const chunkObj = await this.processAIResponse(chunkRespText);
            if (!chunkObj || chunkObj.file !== processedPath) {
              throw new Error(`Unexpected ${messagePrefix}chunk response for ${processedPath}`);
            }
            
            // Unescape and process chunk content
            const contentEscaped = String(chunkObj.content || '');
            const unescaped = contentEscaped.replace(/\\n/g, '\n');
            const processedContent = this.processVariables(unescaped, variables);
            
            // Validate variable replacement
            if (processedContent.includes('{{')) {
              this.outputChannel.appendLine(`WARNING: Unreplaced variables found in ${messagePrefix}chunk ${i} of ${processedPath}`);
            }
            
            // Write chunk directly to file
            await fileHandle.write(processedContent, null, 'utf-8');
            this.sendProgress(`Written ${messagePrefix}chunk ${i}/${total} to ${processedPath}`);
          }
          
          this.sendProgress(`Completed ${messagePrefix}file: ${processedPath}`);
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
        await Promise.all(batch.map(f => 
          generateFileWithChunks(
            f,
            chunksMap,
            (filePath, chunkIndex, totalChunks) => 
              constructFileChunkPrompt(templateContent, manifest, variablesObj, naturalLanguageDescription, filePath, chunkIndex, totalChunks),
            ''
          )
        ));
      }

      this.sendProgress('ROS 2 package files created successfully');

      // Stage 3: Generate tests if requested
      if (testDescription && testDescription.trim() && selectedTestModelId) {
        this.checkCancellation();
        this.sendProgress('Starting test generation...');
        
        // Request test plan
        const testPlanPrompt = this.constructTestPlanPrompt(templateContent, manifest, variablesObj, naturalLanguageDescription, testDescription);
        this.sendProgress('Requesting test generation plan...');
        const testPlanResponseText = await this.sendAIRequest(testPlanPrompt + '\n' + followupInstr);
        const testPlanObj = await this.processAIResponse(testPlanResponseText);
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
          
          // Process test files in parallel batches
          for (let i = 0; i < testFilesList.length; i += MAX_PARALLEL_FILES) {
            const batch = testFilesList.slice(i, i + MAX_PARALLEL_FILES);
            await Promise.all(batch.map(f => 
              generateFileWithChunks(
                f,
                testChunksMap,
                (filePath, chunkIndex, totalChunks) => 
                  this.constructTestFileChunkPrompt(templateContent, manifest, variablesObj, naturalLanguageDescription, testDescription, filePath, chunkIndex, totalChunks),
                'test '
              )
            ));
          }
          
          this.sendProgress('Test files generated successfully');
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
    } finally {
      // Cleanup: destroy SDK session if active
      await this.sdkService.destroySession();
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

  /**
   * Send a request to AI using Copilot SDK
   * Returns the raw text response that needs to be parsed
   */
  private async sendAIRequest(prompt: string): Promise<string> {
    // SDK must be available (no fallback)
    if (!this.sdkService.isSDKAvailable()) {
      throw new Error('Copilot SDK is not available');
    }

    let lastLoggedLength = 0;
    const logInterval = 5000;
    
    const response = await this.sdkService.sendMessage(
      prompt,
      (partialText) => {
        // Progress callback for streaming
        if (partialText.length - lastLoggedLength >= logInterval) {
          this.sendProgress(`Receiving AI response... (${partialText.length} characters so far)`);
          lastLoggedLength = partialText.length;
        }
      }
    );
    
    this.sendProgress(`AI Response received (${response.length} characters)`);
    return response;
  }

  private async processAIResponse(responseText: string): Promise<any> {
    // Clean up the response - remove markdown code blocks if present
    let cleanedResponse = responseText.trim();
    
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
        const prefixIndex = responseText.indexOf(prefix);
        if (prefixIndex !== -1) {
          const afterPrefix = responseText.substring(prefixIndex + prefix.length).trim();
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
      this.sendProgress(`Response preview: ${responseText.substring(0, 200)}...`);
      throw new Error(`AI response is not valid JSON. Response starts with: "${responseText.substring(0, 100)}..."`);
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