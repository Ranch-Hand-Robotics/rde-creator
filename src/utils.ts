import { Uri, Webview } from "vscode";

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

export function getAllManifestMap(extensionUri: Uri) {
  // find all manifest.yaml files in the templates directory
  const templateSource = Uri.joinPath(extensionUri, "templates");
  const manifestFiles = [];
  const fs = require('fs');
  const path = require('path');
  const walkSync = (dir: string, fileMap: Map<string, string>) => {
    fs.readdirSync(dir).forEach((file: string) => {
      const filePath = path.join(dir, file);
      if (fs.statSync(filePath).isDirectory()) {
        fileMap = walkSync(filePath, fileMap);
      } else {
        if (file === "manifest.yaml") {
          // load manifest.yaml, convert to JSON, and add to the map using the directory name
          const yaml = require('js-yaml');
          const contents = fs.readFileSync(filePath, 'utf8');
          const manifest = yaml.load(contents);
          fileMap.set(path.basename(dir), manifest);

        }
      }
    });
    return fileMap;
  };
}

export function fileNameFromVariable(variableValue: string) {
  return variableValue.replace(/ /g, "_").toLowerCase();
}
