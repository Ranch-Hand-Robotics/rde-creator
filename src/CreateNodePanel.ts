import * as vscode from 'vscode';
import { getNonce, getUri, getAllManifestMap, fileNameFromVariable } from './utils';
import { ProcessCreateNode } from './ProcessCreateNode';
import { AIPackageGenerator } from './AIPackageGenerator';
import * as extension from "./extension";

export class CreateNodePanel {
  public static currentPanel: CreateNodePanel | undefined;
  private readonly _panel: vscode.WebviewPanel;
  private _disposables: vscode.Disposable[] = [];
  private readonly _extensionUri: vscode.Uri;
  private _isLoadingManifests: boolean = false;
  private _targetFolderUri?: vscode.Uri;

  

  private constructor(panel: vscode.WebviewPanel, extensionUri: vscode.Uri, targetFolderUri?: vscode.Uri) {
    this._panel = panel;
    this._extensionUri = extensionUri;
    this._targetFolderUri = targetFolderUri;
    this._panel.onDidDispose(() => this.dispose(), null, this._disposables);
    this._panel.webview.html = this._getWebviewContent(this._panel.webview, extensionUri);
    this._setWebviewMessageListener(this._panel.webview);
  }

  private async _loadManifests() {
    // Prevent multiple simultaneous manifest loading operations
    if (this._isLoadingManifests) {
      extension.outputChannel.appendLine('Manifest loading already in progress, skipping...');
      return;
    }

    this._isLoadingManifests = true;

    try {
      extension.outputChannel.appendLine('Loading manifests...');
      
      // Add a timeout to prevent infinite loading
      const timeoutPromise = new Promise((_, reject) => {
        setTimeout(() => reject(new Error('Manifest loading timed out after 30 seconds')), 30000);
      });

      const manifestPromise = getAllManifestMap(vscode.Uri.joinPath(this._extensionUri, 'dist', 'templates'));
      
      const manifests = await Promise.race([manifestPromise, timeoutPromise]) as Map<string, any>;
      const manifestString = JSON.stringify(Object.fromEntries(manifests));
      const message = { command: "setManifests", manifests: manifestString };

      // Debug logging
      extension.outputChannel.appendLine(`Found ${manifests.size} manifests: ${Array.from(manifests.keys()).join(', ')}`);
      extension.outputChannel.appendLine(`Manifest data: ${manifestString}`);

      // Send message to webview
      this._panel.webview.postMessage(message);
      extension.outputChannel.appendLine('Manifests sent to webview');
    } catch (error) {
      extension.outputChannel.appendLine(`Error loading manifests: ${error}`);
      // Send error message to webview
      this._panel.webview.postMessage({ 
        command: "error", 
        text: `Failed to load template manifests. Please consider reinstalling this extension: ${error}` 
      });
    } finally {
      this._isLoadingManifests = false;
    }
  }

  public static render(extensionUri: vscode.Uri, targetFolderUri?: vscode.Uri) {
    if (CreateNodePanel.currentPanel) {
      CreateNodePanel.currentPanel._panel.reveal(vscode.ViewColumn.One);
      // Update target folder if provided
      if (targetFolderUri) {
        CreateNodePanel.currentPanel._targetFolderUri = targetFolderUri;
      }
      // Reload manifests when panel is revealed (only if not already loading)
      if (!CreateNodePanel.currentPanel._isLoadingManifests) {
        CreateNodePanel.currentPanel._loadManifests();
      }
    } else {
      const panel = vscode.window.createWebviewPanel("robotics-templates", "Robotics Tempate", vscode.ViewColumn.One, {
        enableScripts: true,
        localResourceRoots: [vscode.Uri.joinPath(extensionUri, 'dist')]
      });

      CreateNodePanel.currentPanel = new CreateNodePanel(panel, extensionUri, targetFolderUri);
    }
  }

  public dispose() {
    CreateNodePanel.currentPanel = undefined;

    this._panel.dispose();

    while (this._disposables.length) {
      const x = this._disposables.pop();
      if (x) {
        x.dispose();
      }
    }
  }

  private _setWebviewMessageListener(webview: vscode.Webview) {
    webview.onDidReceiveMessage(
      async (message: any) => {
        const command = message.command;

        switch (command) {
          case "webviewLoaded":
            // Webview is ready, now load manifests
            this._loadManifests();
            return;

          case "retryManifests":
            // User clicked retry button, reload manifests
            this._loadManifests();
            return;

          case "createPackage":
            // find the template source directory from the extension package
            const resourceTemplateSource = vscode.Uri.joinPath(this._extensionUri, "dist", "templates", message.type).fsPath;

            // Check if the variables came as a plain object
            if (!message.variables || typeof message.variables !== 'object') {
              vscode.window.showErrorMessage("No variables received from webview");
              return;
            }

            const packageName = message.variables["package_name"];
            if (packageName === undefined) {
              vscode.window.showErrorMessage("Package Name is required");
              return;
            }

            const processCreateNode = new ProcessCreateNode(resourceTemplateSource);
            const variables = new Map<string, string>();

            // Convert the plain object to a Map
            Object.entries(message.variables).forEach(([key, value]) => {
              variables.set(key, value as string);
              
              // add a _file for key
              variables.set(key + "_file", fileNameFromVariable(value as string));
            });

            // Add the current year as a variable for copyright statements
            variables.set("year", new Date().getFullYear().toString());

            // Handle test description for test generation
            const packageTestDescription = message.testDescription || "";
            variables.set("test_description", packageTestDescription);

            const packageFileName = fileNameFromVariable(packageName);
            const workspaceFolder = this._targetFolderUri || vscode.workspace.workspaceFolders?.[0]?.uri;
            if (!workspaceFolder) {
              vscode.window.showErrorMessage("No workspace folder available");
              return;
            }
            const newPackageDir = vscode.Uri.joinPath(workspaceFolder, packageFileName).fsPath;

            extension.outputChannel.appendLine(`Creating package at ${newPackageDir} (target folder: ${this._targetFolderUri?.fsPath || 'workspace root'})`);
            
            try {
              await processCreateNode.createResourcePackage(newPackageDir, variables);
              vscode.window.showInformationMessage(`Package ${packageName} created successfully`);
              this.dispose();
            } catch (error) {
              extension.outputChannel.appendLine(`Error creating package: ${error}`);
              vscode.window.showErrorMessage(`Error creating package: ${error}`);
            }

            return;

          case "createPackageWithAI":
            // AI-powered package generation
            const aiTemplateSource = vscode.Uri.joinPath(this._extensionUri, "dist", "templates", message.type).fsPath;

            if (!message.variables || typeof message.variables !== 'object') {
              vscode.window.showErrorMessage("No variables received from webview");
              return;
            }

            const aiPackageName = message.variables["package_name"];
            if (aiPackageName === undefined) {
              vscode.window.showErrorMessage("Package Name is required");
              return;
            }

            const naturalLanguageDescription = message.naturalLanguageDescription || "";
            if (!naturalLanguageDescription.trim()) {
              vscode.window.showErrorMessage("Please provide a natural language description of what you want to build");
              return;
            }

            const aiVariables = new Map<string, string>();
            Object.entries(message.variables).forEach(([key, value]) => {
              aiVariables.set(key, value as string);
              aiVariables.set(key + "_file", fileNameFromVariable(value as string));
            });
            aiVariables.set("year", new Date().getFullYear().toString());

            // Handle test description for AI generation
            const aiTestDescription = message.testDescription || "";
            aiVariables.set("test_description", aiTestDescription);

            const aiPackageFileName = fileNameFromVariable(aiPackageName);
            const aiWorkspaceFolder = this._targetFolderUri || vscode.workspace.workspaceFolders?.[0]?.uri;
            if (!aiWorkspaceFolder) {
              vscode.window.showErrorMessage("No workspace folder available");
              return;
            }
            const aiNewPackageDir = vscode.Uri.joinPath(aiWorkspaceFolder, aiPackageFileName).fsPath;

            extension.outputChannel.appendLine(`AI-generating package at ${aiNewPackageDir} (target folder: ${this._targetFolderUri?.fsPath || 'workspace root'})`);

            // Run AI generation asynchronously without blocking the UI
            (async () => {
              try {
                const aiGenerator = new AIPackageGenerator(extension.outputChannel, this._panel.webview);
                await aiGenerator.generatePackageWithAI(
                  aiTemplateSource,
                  aiVariables,
                  naturalLanguageDescription,
                  aiNewPackageDir,
                  aiTestDescription
                );
                vscode.window.showInformationMessage(`AI-generated package ${aiPackageName} created successfully`);
                this.dispose();
              } catch (error) {
                extension.outputChannel.appendLine(`AI generation failed: ${error}`);
                vscode.window.showErrorMessage(`AI generation failed: ${error}`);
              }
            })();

            return;

          case "cancel":
            this.dispose();
            return;

          case "error":
            vscode.window.showErrorMessage(message.text);
            return;

          case "info":
            vscode.window.showInformationMessage(message.text);
            return;

          case "trace":
            extension.outputChannel.appendLine(message.text);
            return;
        }
      },
      undefined,
      this._disposables
    );
  }  

  private _getWebviewContent(webview: vscode.Webview, extensionUri: vscode.Uri) {

    const webviewUri = getUri(webview, extensionUri, ["dist", "webview.js"]);
    const styleUri = getUri(webview, extensionUri, ["dist", "style.css"]);

    const nonce = getNonce();
    
    // Tip: Install the es6-string-html VS Code extension to enable code highlighting below
    return /*html*/ `
      <!DOCTYPE html>
      <html lang="en">
        <head>
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">
          <meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${webview.cspSource} 'unsafe-inline'; font-src ${webview.cspSource}; img-src ${webview.cspSource} https:; script-src 'nonce-${nonce}';">
          <link rel="stylesheet" href="${styleUri}">
          <title>ROS 2 Package Creator</title>
        </head>
        <body>
          <div id="root"></div>
          <script nonce="${nonce}" src="${webviewUri}"></script>
        </body>
      </html>
    `;
  }
}