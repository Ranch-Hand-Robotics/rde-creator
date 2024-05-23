import * as vscode from 'vscode';
import { getNonce, getUri, getAllManifestMap, fileNameFromPackage } from './utils';
import { ProcessCreateNode } from './ProcessCreateNode';

export class CreateNodePanel {
  public static currentPanel: CreateNodePanel | undefined;
  private readonly _panel: vscode.WebviewPanel;
  private _disposables: vscode.Disposable[] = [];

  private constructor(panel: vscode.WebviewPanel, extensionUri: vscode.Uri) {
    this._panel = panel;
    this._panel.onDidDispose(() => this.dispose(), null, this._disposables);
    this._panel.webview.html = this._getWebviewContent(this._panel.webview, extensionUri);
    this._setWebviewMessageListener(this._panel.webview);

    let manifests = getAllManifestMap(extensionUri);

    this._panel.webview.postMessage({ command: "setManifests", manifests: manifests});
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
      (message: any) => {
        const command = message.command;

        switch (command) {
          case "createResourcePackage":
            // find the template source directory from the extension package
            const resourceTemplateSource = vscode.Uri.joinPath(vscode.extensions.getExtension("ros2-webview-template")!.extensionUri, "templates", "resource").fsPath;

            const processCreateNode = new ProcessCreateNode(resourceTemplateSource);
            const newPackageDir = vscode.Uri.joinPath(vscode.workspace.workspaceFolders![0].uri, message.packageName).fsPath;
            const variables = new Map<string, string>();
            variables.set("packageName", message.packageName);
            variables.set("packageNameFile", fileNameFromPackage(message.packageName));
            variables.set("packageMaintainer", message.packageMaintainer);
            variables.set("packageVersion", message.packageVersion);
            variables.set("packageDescription", message.packageDescription);
            variables.set("packageLicense", message.packageLicense);
            processCreateNode.createResourcePackage(newPackageDir, variables);


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
              <section class="component-container">
                <h2>Node Properties:</h2>
                <section class="component-example">
                  <vscode-text-field placeholder="node_binary" name="node_name">Node Binary Name</vscode-text-field>
                  <vscode-text-field placeholder="node_class" name="class_name">Primary Class Name</vscode-text-field>
                </section>
              </section>
              <section class="component-container">
                <h2>Include:</h2>
                <section class="component-example">
                  <vscode-checkbox id="include_publisher">Include Publisher</vscode-checkbox>
                  <vscode-checkbox id="include_subscriber">Include Subscriber</vscode-checkbox>
                  <vscode-checkbox id="include_service">Include Service</vscode-checkbox>
                  <vscode-checkbox id="include_action">Include Action</vscode-checkbox>
                </section>
              </section>
              <section class="component-container">
                <h2>Options:</h2>
                <section class="component-example">
                  <vscode-checkbox id="include_i2c">Include I<sup>2</sup>C</vscode-checkbox>
                  <vscode-checkbox id="include_gpio">Include GPIO</vscode-checkbox>
                </section>
              </section>
            </section>
          </div>
          <script type="module" nonce="${nonce}" src="${webviewUri}"></script>
        </body>
      </html>
    `;
  }
}