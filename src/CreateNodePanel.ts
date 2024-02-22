import * as vscode from 'vscode';
import { getNonce, getUri } from './utils';

export class CreateNodePanel {
  public static currentPanel: CreateNodePanel | undefined;
  private readonly _panel: vscode.WebviewPanel;
  private _disposables: vscode.Disposable[] = [];

  private constructor(panel: vscode.WebviewPanel, extensionUri: vscode.Uri) {
    this._panel = panel;
    this._panel.onDidDispose(() => this.dispose(), null, this._disposables);
    this._panel.webview.html = this._getWebviewContent(this._panel.webview, extensionUri);
    this._setWebviewMessageListener(this._panel.webview);
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
        const text = message.text;

        switch (command) {
          case "create_node":
            //vscode.window.showInformationMessage(text);
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
          <title>ROS 2 Node Creator</title>
        </head>
        <body>
          <h1>Create new ROS 2 Package:</h1>
          <section class="component-row">
            <section class="component-container">
            <h2>Package type</h2>
              <section class="component-example">
                <vscode-radio-group orientation="vertical">
                  <label slot="label">Language:</label>
                  <vscode-radio value="type_cpp">C++</vscode-radio>
                  <vscode-radio value="type_python">Python</vscode-radio>
                  <vscode-radio value="type_rust">Rust</vscode-radio>
                  <vscode-radio value="type_resource">Resource</vscode-radio>
                </vscode-radio-group>
              </section>
            </section>

            <section class="component-container">
              <h2>Package Metadata</h2>
              <section class="component-example">
                <vscode-text-field placeholder="my_package" name="package_name">Name</vscode-text-field>
                <vscode-text-field placeholder="nobody@nowhere.robots" name="package_maintainer">Maintainer</vscode-text-field>
                <vscode-text-field placeholder="0.0.0" name="package_version">Version</vscode-text-field>
                <vscode-text-area placeholder="This is a sample description" rows="20" name="package_description">Description</vscode-text-area>
                <p>License:</p>
                <vscode-dropdown position="below">
                  <vscode-option name="MIT">MIT</vscode-option>
                  <vscode-option name="Apache">Apache</vscode-option>
                  <vscode-option name="Other">Other</vscode-option>
                </vscode-dropdown>
              </section>
            </section>
          </section>
          <section id="component-row">
            <vscode-button id="create_node">Create Node</vscode-button>
          </section>
          <script type="module" nonce="${nonce}" src="${webviewUri}"></script>
        </body>
      </html>
    `;
  }
}