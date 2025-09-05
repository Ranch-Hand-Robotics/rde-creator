import * as vscode from 'vscode';
import { getNonce, getUri, getAllManifestMap, fileNameFromVariable } from './utils';
import { ProcessCreateNode } from './ProcessCreateNode';
import * as extension from "./extension";

export class CreateNodePanel {
  public static currentPanel: CreateNodePanel | undefined;
  private readonly _panel: vscode.WebviewPanel;
  private _disposables: vscode.Disposable[] = [];
  private readonly _extensionUri: vscode.Uri;

  

  private constructor(panel: vscode.WebviewPanel, extensionUri: vscode.Uri) {
    this._panel = panel;
    this._extensionUri = extensionUri;
    this._panel.onDidDispose(() => this.dispose(), null, this._disposables);
    this._panel.webview.html = this._getWebviewContent(this._panel.webview, extensionUri);
    this._setWebviewMessageListener(this._panel.webview);

    // Load manifests asynchronously
    this._loadManifests();
  }

  private async _loadManifests() {
    try {
      const manifests = await getAllManifestMap(vscode.Uri.joinPath(this._extensionUri, 'dist', 'templates'));
      const manifestString = JSON.stringify(Object.fromEntries(manifests));
      const message = { command: "setManifests", manifests: manifestString };

      // Debug logging
      extension.outputChannel.appendLine(`Found ${manifests.size} manifests: ${Array.from(manifests.keys()).join(', ')}`);
      extension.outputChannel.appendLine(`Manifest data: ${manifestString}`);

      this._panel.webview.postMessage(message);
    } catch (error) {
      extension.outputChannel.appendLine(`Error loading manifests: ${error}`);
    }
  }

  public static render(extensionUri: vscode.Uri) {
    if (CreateNodePanel.currentPanel) {
      CreateNodePanel.currentPanel._panel.reveal(vscode.ViewColumn.One);
    } else {
      const panel = vscode.window.createWebviewPanel("robotics-templates", "Robotics Tempate", vscode.ViewColumn.One, {
        enableScripts: true,
        localResourceRoots: [vscode.Uri.joinPath(extensionUri, 'dist')]
      });

      CreateNodePanel.currentPanel = new CreateNodePanel(panel, extensionUri);
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

            const packageFileName = fileNameFromVariable(packageName);
            const newPackageDir = vscode.Uri.joinPath(vscode.workspace.workspaceFolders![0].uri, packageFileName).fsPath;

            extension.outputChannel.appendLine(`Creating package at ${newPackageDir} with ${variables.size} variables`);
            
            try {
              await processCreateNode.createResourcePackage(newPackageDir, variables);
              vscode.window.showInformationMessage(`Package ${packageName} created successfully`);
              this.dispose();
            } catch (error) {
              extension.outputChannel.appendLine(`Error creating package: ${error}`);
              vscode.window.showErrorMessage(`Error creating package: ${error}`);
            }

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