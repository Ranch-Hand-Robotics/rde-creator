import fs from 'fs';
import path from 'path';
import yaml from 'yaml';
import Handlebars from 'handlebars';

// This file implements the ProcessCreateNode class. It's purpose is to handle the creation of a new ROS 2 node in the extension. 

export class ProcessCreateNode {
  private nodeTemplateSource: string;

  constructor(nodeTemplateSource: string) {
    this.nodeTemplateSource = nodeTemplateSource;
  }


  // Creates Resource package.
  public createResourcePackage(newPackageDir: string, variables: Map<string, string>) {
    const manifestFile = path.join(this.nodeTemplateSource, 'manifest.yaml');

    // Read the manifest file
    const manifestContent = fs.readFileSync(manifestFile, 'utf8');

    const sampleManifest =`
    name: "template friendly name"
    version: "0.0.0"
    description: "A templateMapping package"
    maintainers:
    email:
    license: "MIT"
    options:
      - include_urdf:
        name: "Include URDF"
        description: "Include a URDF file in the package"
        type: "boolean"
    files:
      - "urdf":
        condition: include_urdf
        files:
          - source: "t.urdf"
            destination: "r.urdf"
      - package_name: {{package_name}}
`;

    const manifest = yaml.parse(manifestContent);

    console.log(` Parsing: ${manifestContent}`);


    // Copy the files from the template directory to the new package directory
    
    const copyFilesRecursively = (source: string, destination: string, fileMapping: Array<object> | undefined) => {
      // Get all files and directories in the source directory
      const files = fs.readdirSync(source);

      // create destination directory if it doesn't exist
      if (!fs.existsSync(destination)) {
        fs.mkdirSync(destination);
      }

      // recursively through each file/directory
      files.forEach((filename : string) => {
        console.log(` Processing: ${filename}`);

        const sourcePath = path.join(source, filename);
        let destinationPath = path.join(destination, filename);
        let newMapping : undefined | object | Array<object> = undefined;
        let condition : undefined | string = undefined;
        if (fileMapping !== undefined) {
          newMapping = fileMapping.find((element: any) => {
            // On element, lookup the property by string
            return Object.keys(element).includes(filename);
          });
        }

        if (newMapping !== undefined) {

          console.log(` Found mapping for file ${filename}: ${JSON.stringify(newMapping)}`);

          // Get the new filename
          const key = filename as keyof typeof newMapping;

          // if this is a yaml object, it is interpreted as a directory, potentially with a condition
          // it has the format:
          // filename: destination_file
          //  destination:  
          //  condition: condition_name
          //  files:
          //  - filename: destination_file
          //    condition: condition_name
          if (newMapping[key] !== null && typeof newMapping[key] === 'string') {
            destinationPath = path.join(destination, newMapping[key]);
            console.log(` Simple mapping ${filename} -> ${destinationPath}`);
          }

          // Check if the directory has a condition
          const conditionKey = "condition" as keyof typeof newMapping;
          if (newMapping[conditionKey]) {
            // Check if the condition is met
            condition = newMapping[conditionKey] as string;
            console.log(` Found Condition: ${condition}`);
          }
        }
      
        if (condition) {
          if (!variables.has(condition) || variables.get(condition) === "false") {
            console.log(` Skipping ${filename} due to condition ${condition} not being specified or false.`);
            return;
          }
        }

        // process the filename to replace variables using handlebar semantics
        for (const [key, value] of variables) {
          destinationPath = destinationPath.replace(`{{${key}}}`, value);
        }



        // Check if it's a directory
        if (fs.statSync(sourcePath).isDirectory()) {
          if (newMapping !== undefined) {
            const filesKey = "files" as keyof typeof newMapping;
            fileMapping = newMapping[filesKey];
          }


          // Create the corresponding directory in the destination
          fs.mkdirSync(destinationPath);

          // Recursively copy files from the subdirectory
          copyFilesRecursively(sourcePath, destinationPath, fileMapping);
        } else {

          // Read the template file
          const templateContent = fs.readFileSync(sourcePath, 'utf8');

          // load the template using handlebars.js
          const template = Handlebars.compile(templateContent);

          // Convert the variables map to an object with variables as members
          const variablesObject = Object.fromEntries(variables);

          const result = template(variablesObject);

          console.log(` Writing: ${destinationPath}`);

          // Write the file to the destination
          fs.writeFileSync(destinationPath, result);
        }
      });
    };

    // Copy the files from the template directory to the new package directory
    copyFilesRecursively(this.nodeTemplateSource, newPackageDir, manifest.files);
  }

  
}
