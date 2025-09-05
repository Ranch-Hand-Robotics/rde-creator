import { Uri, Webview, workspace, RelativePattern } from "vscode";
import * as extension from "./extension";

export function getNonce() {
  let text = "";
  const possible = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  for (let i = 0; i < 32; i++) {
    text += possible.charAt(Math.floor(Math.random() * possible.length));
  }
  return text;
}

export function getUri(webview: Webview, extensionUri: Uri, pathList: string[]) {
  return webview.asWebviewUri(Uri.joinPath(extensionUri, ...pathList));
}

export async function getAllManifestMap(templateSource: Uri): Promise<Map<string, any>> {
  const manifestFiles = new Map<string, any>();

  try {
    // Use VS Code workspace filesystem API instead of Node.js fs
    const files = await workspace.findFiles(
      new RelativePattern(templateSource, '**/manifest.yaml'),
      null,
      100
    );

    extension.outputChannel.appendLine(`Found ${files.length} manifest files`);

    for (const file of files) {
      try {
        const content = await workspace.fs.readFile(file);
        const text = Buffer.from(content).toString('utf8');

        // Parse YAML using the yaml package (not js-yaml)
        const yaml = require('yaml');
        const manifest = yaml.parse(text);

        // Get the directory name (template name)
        const dirName = workspace.asRelativePath(file.path).split('/').slice(-2)[0];
        manifestFiles.set(dirName, manifest);

        extension.outputChannel.appendLine(`Loaded manifest for ${dirName}: ${manifest.name}`);
      } catch (e) {
        extension.outputChannel.appendLine(`Error parsing manifest ${file.path}: ${e}`);
      }
    }
  } catch (e) {
    extension.outputChannel.appendLine(`Error finding manifest files: ${e}`);
  }

  extension.outputChannel.appendLine(`Total manifests loaded: ${manifestFiles.size}`);
  return manifestFiles;
}

export function fileNameFromVariable(variableValue: string) {
  return variableValue.replace(/ /g, "_").toLowerCase();
}
