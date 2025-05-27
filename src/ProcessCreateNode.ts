import fs from 'fs/promises';
import path from 'path';
import yaml from 'yaml';
import Handlebars from 'handlebars';
import * as extension from "./extension";

// This file implements the ProcessCreateNode class. It's purpose is to handle the creation of a new ROS 2 node in the extension. 

interface CopyQueueItem {
  source: string;
  destination: string;
  fileMapping?: Array<object>;
}

export class ProcessCreateNode {
  private nodeTemplateSource: string;

  constructor(nodeTemplateSource: string) {
    this.nodeTemplateSource = nodeTemplateSource;
  }

  // Creates Resource package.
  public async createResourcePackage(newPackageDir: string, variables: Map<string, string>) {
    const manifestFile = path.join(this.nodeTemplateSource, 'manifest.yaml');

    // Read the manifest file directly
    const manifestContent = await fs.readFile(manifestFile, 'utf-8');
    const manifest = yaml.parse(manifestContent);

    extension?.outputChannel?.appendLine(` Parsing: ${manifestContent}`);

    // Copy the files from the template directory to the new package directory using iterative approach
    await this.copyFilesIteratively(this.nodeTemplateSource, newPackageDir, manifest.files, variables);
  }

  private async copyFilesIteratively(
    sourceRoot: string, 
    destinationRoot: string, 
    fileMapping: Array<object> | undefined,
    variables: Map<string, string>
  ) {
    // Create a queue for BFS traversal
    const queue: CopyQueueItem[] = [
      { source: sourceRoot, destination: destinationRoot, fileMapping }
    ];
    
    // Process items in queue one by one
    while (queue.length > 0) {
      const { source, destination, fileMapping } = queue.shift()!;
      
      // Create destination directory if it doesn't exist
      try {
        await fs.mkdir(destination, { recursive: true });
      } catch (error) {
        // Directory might already exist, which is fine
        if ((error as NodeJS.ErrnoException).code !== 'EEXIST') {
          throw error;
        }
      }
      
      // Get all files and directories in the source directory
      const files = await fs.readdir(source);
      
      // Process each file/directory
      for (const filename of files) {
        const sourcePath = path.join(source, filename);
        let destinationPath = path.join(destination, filename);
        let newMapping: undefined | object | Array<object> = undefined;
        let condition: undefined | string = undefined;
          // Check if there's a mapping for this file
        if (fileMapping !== undefined) {
          newMapping = fileMapping.find((element: any) => {
            return Object.keys(element).includes(filename);
          });
        }
          if (newMapping !== undefined) {
          extension.outputChannel?.appendLine(` Found mapping for file ${filename}: ${JSON.stringify(newMapping)}`);
          console.log(` Found mapping for file ${filename}: ${JSON.stringify(newMapping)}`);
          
          // Get the new filename
          const key = filename as keyof typeof newMapping;
          
          // If this is a string mapping, use it directly
          if (newMapping[key] !== null && typeof newMapping[key] === 'string') {
            destinationPath = path.join(destination, newMapping[key]);
            extension.outputChannel?.appendLine(` Simple mapping ${filename} -> ${destinationPath}`);
            console.log(` Simple mapping ${filename} -> ${destinationPath}`);
          }
          
          // Check if the directory has a condition
          const conditionKey = "condition" as keyof typeof newMapping;
          if (newMapping[conditionKey]) {
            if (Array.isArray(newMapping[conditionKey])) {
              // Handle array of conditions according to specification
              const conditions = newMapping[conditionKey] as string[];
              condition = conditions.length > 0 ? conditions[0] : undefined;
              extension.outputChannel?.appendLine(` Found Conditions: ${conditions.join(', ')}`);
              console.log(` Found Conditions: ${conditions.join(', ')}`);
            } else {
              condition = newMapping[conditionKey] as string;
              extension.outputChannel?.appendLine(` Found Condition: ${condition}`);
              console.log(` Found Condition: ${condition}`);
            }
          }
        }
          // Skip if condition is not met
        if (condition) {
          // Check if the condition variable exists and is not "false"
          if (!variables.has(condition) || variables.get(condition) === "false") {
            extension.outputChannel?.appendLine(` Skipping ${filename} due to condition ${condition} not being specified or false.`);
            continue;
          }
        }
        
        // Process filename with handlebar variables
        for (const [key, value] of variables) {
          destinationPath = destinationPath.replace(`{{${key}}}`, value);
        }
        
        // Check if it's a directory
        const stats = await fs.stat(sourcePath);
        if (stats.isDirectory()) {
          let nextFileMapping = undefined;
          if (newMapping !== undefined) {
            const filesKey = "files" as keyof typeof newMapping;
            nextFileMapping = newMapping[filesKey];
          }
          
          // Add directory to queue for processing
          queue.push({
            source: sourcePath,
            destination: destinationPath,
            fileMapping: nextFileMapping
          });        } else {
          // Process file
          extension.outputChannel?.appendLine(` Processing file: ${sourcePath} -> ${destinationPath}`);
          
          // Read the template file
          const templateContent = await fs.readFile(sourcePath, 'utf8');
          
          // Load the template using handlebars.js
          const template = Handlebars.compile(templateContent);
          
          // Convert the variables map to an object
          const variablesObject = Object.fromEntries(variables);
          
          // Apply template
          const result = template(variablesObject);
          
          extension.outputChannel?.appendLine(` Writing: ${destinationPath}`);
          
          // Write the file to the destination
          await fs.writeFile(destinationPath, result);
        }
      }
    }
  }
}
