import * as vscode from 'vscode';
import { CreateNodePanel } from './CreateNodePanel';

export let outputChannel: vscode.OutputChannel;

export function activate(context: vscode.ExtensionContext) {

  outputChannel = vscode.window.createOutputChannel("Robotics Templates");
  context.subscriptions.push(outputChannel);

  outputChannel.appendLine('"Robotics Templates" is now active!');

  const createNodeCommand = vscode.commands.registerCommand("robotics-templates.create", () => {
    CreateNodePanel.render(context.extensionUri);
  });

  context.subscriptions.push(createNodeCommand);

}


export function deactivate() {

}
