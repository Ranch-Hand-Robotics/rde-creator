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

    let manifests = getAllManifestMap(vscode.Uri.joinPath(extensionUri, 'dist', 'templates'));

    let manifestString = JSON.stringify(Object.fromEntries(manifests));
    let message = { command: "setManifests", manifests: manifestString};

    this._panel.webview.postMessage(message);
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
          <div id="create_package_page">
            <h1>Create new ROS 2 Package:</h1>
            <section class="component-row">
              <section class="component-container">
              <h2>Package type</h2>
                <section class="component-example">
                  <vscode-radio-group id="ros_package_type" orientation="vertical">
                    <label slot="label">Package Type:</label>
                  </vscode-radio-group>
                </section>
              </section>
              <section class="component-container">
                <h2>Package Metadata</h2>
                <section class="component-example">
                  <vscode-text-field placeholder="my_package" id="package_name">Name</vscode-text-field>
                  <vscode-text-field placeholder="nobody@nowhere.robots" id="package_maintainer">Maintainer</vscode-text-field>
                  <vscode-text-field placeholder="0.0.0" id="package_version">Version</vscode-text-field>
                  <vscode-text-area placeholder="This is a sample description" rows="5" cols="50" id="package_description">Description</vscode-text-area>
                  <p>License:</p>
                  <vscode-text-field placeholder="MIT" id="package_license">License</vscode-text-field>
                </section>
              </section>
            </section>
            <section id="component-row">
              <vscode-button id="second_page_button">Next Page</vscode-button>
            </section>
          </div>
          <!-- Hidden by default -->
          <div id="create_node_page" class="hidden">
            <h1>Populate ROS 2 Node:</h1>
            <section class="component-row">
              <section class="component-container"  id="IncludeContainer">
              </section>
            </section>
            <section id="component-row">
              <vscode-button id="create_node_button">Create</vscode-button>
            </section>
          </div>
          <script type="module" nonce="${nonce}" src="${webviewUri}"></script>
        </body>
      </html>
    `;
  }
}