import fs from 'fs';
import path from 'path';
import yaml from 'yaml';
import Handlebars from 'handlebars';

// This file implements the ProcessCreateNode class. It's purpose is to handle the creation of a new ROS 2 node in the extension. 

export class ProcessCreateNode {
  private packageName: string;
  private nodeName: string;
  private nodeTemplateSource: string;

  constructor(packageName: string, nodeName: string, nodeTemplateSource: string) {
    this.packageName = packageName;
    this.nodeName = nodeName;
    this.nodeTemplateSource = nodeTemplateSource;
  }

  // Creates Resource package.
  public createResourcePackage(newPackageDir: string, variables: Map<string, string>) {
    const manifestFile = path.join(this.nodeTemplateSource, 'manifest.yaml');

    // Read the manifest file
    const manifestContent = fs.readFileSync(manifestFile, 'utf8');

    // Parse the manifest. It has the following format:
    // name: template friendly name
    // version: 0.0.0
    // description: A templateMapping package
    // maintainers:
    //   - name: templateMapping
    //     email:
    // license: Apache 2.0
    // file_mapping:
    //   - TemplateNode.cpp: {{variable1}}Node.cpp
    //   - TemplateNode.hpp: {{variable2}}Node.hpp
    //   - CMakeLists.txt: CMakeLists.txt
    //   - package.xml: package.xml
    const manifest = yaml.parse(manifestContent);

    // Copy the files from the template directory to the new package directory
    
    const copyFilesRecursively = (source: string, destination: string) => {
      // Get all files and directories in the source directory
      const files = fs.readdirSync(source);

      // create destination directory if it doesn't exist
      if (!fs.existsSync(destination)) {
        fs.mkdirSync(destination);
      }

      // Iterate through each file/directory
      files.forEach((filename : string) => {
        const sourcePath = path.join(source, filename);
        const destinationPath = path.join(destination, filename);

        // Check if it's a directory
        if (fs.statSync(sourcePath).isDirectory()) {
          // Create the corresponding directory in the destination
          fs.mkdirSync(destinationPath);

          // Recursively copy files from the subdirectory
          copyFilesRecursively(sourcePath, destinationPath);
        } else {

          const newMapping = manifest.file_mapping.find((element: any) => {
            // On element, lookup the property by string
            return Object.keys(element).includes(filename);
          });

          if (newMapping !== undefined) {
            // Get the new filename
            const key = filename as keyof typeof newMapping;
            filename = newMapping[key];

            // process the filename to replace variables using handlebar semantics
            for (const [key, value] of variables) {
              filename = filename.replace(`{{${key}}}`, value);
            }
          }

          // Read the template file
          const templateContent = fs.readFileSync(sourcePath, 'utf8');

          // load the template using handlebars.js
          const template = Handlebars.compile(templateContent);

          // Convert the variables map to an object with variables as members
          const variablesObject = Object.fromEntries(variables);

          const result = template(variablesObject);

          const filePath = path.join(destination, filename);

          // Write the file to the destination
          fs.writeFileSync(filePath, result);
        }
      });
    };

    // Copy the files from the template directory to the new package directory
    copyFilesRecursively(this.nodeTemplateSource, newPackageDir);
  }

  
}
