import * as vscode from 'vscode';
import { CreateNodePanel } from './CreateNodePanel';

export let outputChannel: vscode.OutputChannel;

export function activate(context: vscode.ExtensionContext) {

  outputChannel = vscode.window.createOutputChannel("Robotics Templates");
  context.subscriptions.push(outputChannel);

  outputChannel.appendLine('"Robotics Templates" is now active!');

  const createNodeCommand = vscode.commands.registerCommand("robotics-templates.create", (uri?: vscode.Uri) => {
    outputChannel.appendLine(`Creating ROS package. Target folder: ${uri?.fsPath || 'workspace root'}`);
    CreateNodePanel.render(context.extensionUri, uri);
  });

  const buildWorkspaceCommand = vscode.commands.registerCommand("robotics-templates.buildWorkspace", async () => {
    outputChannel.appendLine('Building ROS 2 workspace...');
    
    const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
    if (!workspaceFolder) {
      vscode.window.showErrorMessage('No workspace folder found');
      return;
    }

    const terminal = vscode.window.createTerminal({
      name: 'ROS 2 Build',
      cwd: workspaceFolder.uri.fsPath
    });
    
    terminal.show();
    terminal.sendText('colcon build');
    
    outputChannel.appendLine(`Building workspace at: ${workspaceFolder.uri.fsPath}`);
  });

  const sourceWorkspaceCommand = vscode.commands.registerCommand("robotics-templates.sourceWorkspace", async () => {
    outputChannel.appendLine('Sourcing ROS 2 workspace...');
    
    const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
    if (!workspaceFolder) {
      vscode.window.showErrorMessage('No workspace folder found');
      return;
    }

    const terminal = vscode.window.createTerminal({
      name: 'ROS 2 Source',
      cwd: workspaceFolder.uri.fsPath
    });
    
    terminal.show();
    
    // Check if install/setup.bash exists, otherwise try install/setup.sh
    const setupBash = vscode.Uri.joinPath(workspaceFolder.uri, 'install', 'setup.bash');
    const setupSh = vscode.Uri.joinPath(workspaceFolder.uri, 'install', 'setup.sh');
    
    try {
      await vscode.workspace.fs.stat(setupBash);
      terminal.sendText('source install/setup.bash');
      outputChannel.appendLine('Sourced install/setup.bash');
    } catch {
      try {
        await vscode.workspace.fs.stat(setupSh);
        terminal.sendText('source install/setup.sh');
        outputChannel.appendLine('Sourced install/setup.sh');
      } catch {
        terminal.sendText('# No setup file found. Please build the workspace first with "colcon build"');
        vscode.window.showWarningMessage('No setup file found. Please build the workspace first.');
        outputChannel.appendLine('No setup file found in install/ directory');
      }
    }
  });

  const cleanWorkspaceCommand = vscode.commands.registerCommand("robotics-templates.cleanWorkspace", async () => {
    outputChannel.appendLine('Cleaning ROS 2 workspace...');
    
    const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
    if (!workspaceFolder) {
      vscode.window.showErrorMessage('No workspace folder found');
      return;
    }

    const result = await vscode.window.showWarningMessage(
      'This will remove build, install, and log directories. Continue?',
      'Yes', 'No'
    );

    if (result === 'Yes') {
      const terminal = vscode.window.createTerminal({
        name: 'ROS 2 Clean',
        cwd: workspaceFolder.uri.fsPath
      });
      
      terminal.show();
      terminal.sendText('rm -rf build install log');
      
      outputChannel.appendLine(`Cleaned workspace at: ${workspaceFolder.uri.fsPath}`);
    }
  });

  const listPackagesCommand = vscode.commands.registerCommand("robotics-templates.listPackages", async () => {
    outputChannel.appendLine('Listing ROS 2 packages...');
    
    const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
    if (!workspaceFolder) {
      vscode.window.showErrorMessage('No workspace folder found');
      return;
    }

    const terminal = vscode.window.createTerminal({
      name: 'ROS 2 Packages',
      cwd: workspaceFolder.uri.fsPath
    });
    
    terminal.show();
    terminal.sendText('colcon list');
    
    outputChannel.appendLine(`Listing packages in workspace: ${workspaceFolder.uri.fsPath}`);
  });

  context.subscriptions.push(createNodeCommand, buildWorkspaceCommand, sourceWorkspaceCommand, cleanWorkspaceCommand, listPackagesCommand);

}


export function deactivate() {

}
