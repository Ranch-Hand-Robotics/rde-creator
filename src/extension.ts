// Copyright (c) Ranch Hand Robotics, LLC. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import { CreateNodePanel } from './CreateNodePanel';

export let outputChannel: vscode.OutputChannel;

export function activate(context: vscode.ExtensionContext) {

  outputChannel = vscode.window.createOutputChannel("RDE Creator");
  context.subscriptions.push(outputChannel);

  outputChannel.appendLine('"RDE Creator" is now active!');

  const createNodeCommand = vscode.commands.registerCommand("robotics-templates.create", (uri?: vscode.Uri) => {
    outputChannel.appendLine(`Creating ROS package. Target folder: ${uri?.fsPath || 'workspace root'}`);
    CreateNodePanel.render(context.extensionUri, uri);
  });
}

export function deactivate() {

}
