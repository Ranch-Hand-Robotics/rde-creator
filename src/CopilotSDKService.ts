// Copyright (c) Ranch Hand Robotics, LLC. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';

// Dynamic import types for Copilot SDK
type CopilotClient = any;
type CopilotSession = any;

/**
 * Service for interacting with GitHub Copilot via the Copilot SDK.
 * The SDK is bundled with the extension as an npm dependency.
 */
export class CopilotSDKService {
  private client: CopilotClient | undefined;
  private session: CopilotSession | undefined;
  private outputChannel: vscode.OutputChannel;
  private sdkAvailable: boolean = false;
  private CopilotSDK: any; // Dynamically imported module

  constructor(outputChannel: vscode.OutputChannel) {
    this.outputChannel = outputChannel;
  }

  /**
   * Initialize the Copilot SDK client.
   * The SDK and CLI are bundled with the extension.
   * @throws Error if SDK initialization fails
   */
  async initialize(): Promise<void> {
    try {
      this.outputChannel.appendLine('Initializing GitHub Copilot SDK...');
      
      // Dynamically import the SDK (ES module)
      this.CopilotSDK = await import('@github/copilot-sdk');
      
      // Create client with default options (uses bundled CLI via stdio)
      this.client = new this.CopilotSDK.CopilotClient({
        useStdio: true, // Use stdio for better compatibility
        logLevel: 'error',
        autoStart: true,
        autoRestart: true
      });

      // Start the client and verify connection
      await this.client.start();
      
      // Ping to verify connectivity
      await this.client.ping('initialization check');
      
      this.sdkAvailable = true;
      this.outputChannel.appendLine('GitHub Copilot SDK initialized successfully');
    } catch (error) {
      this.outputChannel.appendLine(`Failed to initialize Copilot SDK: ${error}`);
      this.sdkAvailable = false;
      this.client = undefined;
      throw new Error(`GitHub Copilot SDK failed to initialize: ${error}`);
    }
  }



  /**
   * Check if the SDK is available and initialized
   */
  isSDKAvailable(): boolean {
    return this.sdkAvailable && this.client !== undefined;
  }

  /**
   * Create a new session for conversation
   */
  async createSession(model?: string): Promise<CopilotSession | undefined> {
    if (!this.isSDKAvailable()) {
      return undefined;
    }

    try {
      this.outputChannel.appendLine(`Creating Copilot session${model ? ` with model: ${model}` : ''}`);
      
      this.session = await this.client!.createSession({
        model: model || undefined
      });
      
      this.outputChannel.appendLine(`Session created: ${this.session.sessionId}`);
      return this.session;
    } catch (error) {
      this.outputChannel.appendLine(`Failed to create session: ${error}`);
      return undefined;
    }
  }

  /**
   * Send a message and get streaming response
   * @param prompt The prompt to send
   * @param onProgress Callback for progress updates (streaming text)
   * @param onComplete Callback when response is complete
   * @returns Promise that resolves with the complete response
   */
  async sendMessage(
    prompt: string,
    onProgress?: (text: string) => void,
    onComplete?: () => void
  ): Promise<string> {
    if (!this.session) {
      throw new Error('No active session. Call createSession() first.');
    }

    return new Promise((resolve, reject) => {
      let fullResponse = '';
      
      // Subscribe to session events
      const unsubscribe = this.session!.on((event: any) => {
        try {
          if (event.type === 'assistant.message') {
            fullResponse = event.data.content;
            if (onProgress) {
              onProgress(fullResponse);
            }
          } else if (event.type === 'session.error') {
            this.outputChannel.appendLine(`Session error: ${event.data.message}`);
            unsubscribe(); // Clean up subscription
            reject(new Error(event.data.message));
          }
        } catch (error) {
          unsubscribe(); // Clean up on any error
          reject(error);
        }
      });

      // Send message and wait for completion
      this.session!.sendAndWait({ prompt }, 120000) // 2 minute timeout
        .then(() => {
          unsubscribe();
          if (onComplete) {
            onComplete();
          }
          resolve(fullResponse);
        })
        .catch((error: any) => {
          unsubscribe();
          reject(error);
        });
    });
  }

  /**
   * Abort the current session's message processing
   */
  async abort(): Promise<void> {
    if (this.session) {
      try {
        await this.session.abort();
        this.outputChannel.appendLine('Session aborted');
      } catch (error) {
        this.outputChannel.appendLine(`Failed to abort session: ${error}`);
      }
    }
  }

  /**
   * Destroy the current session
   */
  async destroySession(): Promise<void> {
    if (this.session) {
      try {
        await this.session.destroy();
        this.outputChannel.appendLine('Session destroyed');
        this.session = undefined;
      } catch (error) {
        this.outputChannel.appendLine(`Failed to destroy session: ${error}`);
      }
    }
  }

  /**
   * Cleanup and shutdown the SDK client
   */
  async shutdown(): Promise<void> {
    if (this.session) {
      await this.destroySession();
    }

    if (this.client) {
      try {
        this.outputChannel.appendLine('Shutting down Copilot SDK client...');
        const errors = await this.client.stop();
        if (errors.length > 0) {
          this.outputChannel.appendLine(`Shutdown completed with ${errors.length} errors`);
          errors.forEach((err: any) => this.outputChannel.appendLine(`  - ${err.message}`));
        } else {
          this.outputChannel.appendLine('Copilot SDK client shut down successfully');
        }
        this.client = undefined;
        this.sdkAvailable = false;
      } catch (error) {
        this.outputChannel.appendLine(`Failed to shutdown client: ${error}`);
      }
    }
  }
}
