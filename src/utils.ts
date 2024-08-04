import { Uri, Webview } from "vscode";
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

export function getAllManifestMap(templateSource: Uri): Map<string, string> {
  const manifestFiles = new Map<string, string>();
  const fs = require('fs');
  const path = require('path');
  const walkSync = (dir: string, fileMap: Map<string, string>) => {
    fs.readdirSync(dir).forEach((file: string) => {
      const filePath = path.join(dir, file);
      if (fs.statSync(filePath).isDirectory()) {
        walkSync(filePath, fileMap);
      } else {
        if (file === "manifest.yaml") {
          try {
            // load manifest.yaml, convert to JSON, and add to the map using the directory name
            const yaml = require('js-yaml');
            const contents = fs.readFileSync(filePath, 'utf8');
            const manifest = yaml.load(contents);
            fileMap.set(path.basename(dir), manifest);
          } catch (e) {
            extension.outputChannel.appendLine(`Error reading manifest.yaml in ${dir}: ${e}`);
          }
        }
      }
    });
    return fileMap;
  };

  walkSync(templateSource.fsPath, manifestFiles);

  return manifestFiles;
}

export function fileNameFromVariable(variableValue: string) {
  return variableValue.replace(/ /g, "_").toLowerCase();
}
