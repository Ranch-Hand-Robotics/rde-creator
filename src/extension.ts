import * as vscode from 'vscode';
import { CreateNodePanel } from './CreateNodePanel';

export function activate(context: vscode.ExtensionContext) {

  console.log('"robotics-templates" is now active!');

  const createNodeCommand = vscode.commands.registerCommand("robotics-templates.create", () => {
    CreateNodePanel.render(context.extensionUri);
  });

  context.subscriptions.push(createNodeCommand);
}


export function deactivate() {

}
