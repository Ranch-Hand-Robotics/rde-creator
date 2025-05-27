import * as assert from 'assert';

// You can import and use all API from the 'vscode' module
// as well as import your extension to test it
import * as vscode from 'vscode';
// import * as myExtension from '../../extension';
import {ProcessCreateNode} from '../../ProcessCreateNode';
import path from 'path';
import fs from 'fs';

// Create a new test suite for the ProcessCreateNode class
suite('ProcessCreateNode Test Suite', () => {
  // Test the createResourcePackage method
  test('createResourcePackage Test', async () => {

    // Use mock-fs to mock the manifest.yaml file, a single cpp file, a single hpp file, a CMakeLists.txt file, and a package.xml file
    const mock = require('mock-fs');
    mock({
      'test_source': {
        'manifest.yaml': `
name: "template friendly name"
version: 0.0.0
description: A templateMapping package
maintainers:
  - name: templateMapping
email:
license: "Apache 2.0"
files:
  - TemplateNode.cpp: "{{variable1}}Node.cpp"
  - TemplateNode.hpp: "{{variable2}}Node.hpp"
  - CMakeLists.txt: CMakeLists.txt
  - package.xml: package.xml
`,
        'TemplateNode.cpp': 'class {{class_name}} {\n}\n',
        'CMakeLists.txt': 'cmake_minimum_required(VERSION 3.5)\nproject({{variable1}})\n\nfind_package(ament_cmake REQUIRED)\n\nament_package()',
        'package.xml': '<?xml version="1.0"?>\n<package format="2">\n  <name>{{variable1}}</name>\n  <version>0.0.0</version>\n  <description>Template package</description>\n  <maintainer email="" name="templateMapping"></maintainer>\n  <license>Apache 2.0</license>\n</package>'
      }
    });


    const templateSource = path.join(process.cwd(), "test_source");
    
    const processCreateNode = new ProcessCreateNode(templateSource);

    const newPackageDir = path.join(process.cwd(), "test_package");

    // Create a new map of variables
    const variables = new Map<string, string>();
    variables.set("variable1", "test");
    variables.set("variable2", "test2");
    variables.set("class_name", "test_class");

    // Call the createResourcePackage method
    await processCreateNode.createResourcePackage(newPackageDir, variables);
    // Check if the package directory was created
    assert.strictEqual(fs.existsSync(newPackageDir), true);
    // Check if the package directory contains the correct files
    assert.strictEqual(fs.existsSync(path.join(newPackageDir, "CMakeLists.txt")), true);
    assert.strictEqual(fs.existsSync(path.join(newPackageDir, "package.xml")), true);
    assert.strictEqual(fs.existsSync(path.join(newPackageDir, "testNode.cpp")), true);

    mock.restore();
  });

  test('createResourcePackage with subdirectory Test', async () => {

    // Use mock-fs to mock the manifest.yaml file, a single cpp file, a single hpp file, a CMakeLists.txt file, and a package.xml file
    const mock = require('mock-fs');
    let mockInit = {
      'test_source': {
        'manifest.yaml': `
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
    default: false
files:
  - "urdf":
    condition: include_urdf
  - "urdf/t.urdf": "urdf/r.urdf"
    condition: include_urdf
  - package.xml: "package.xml"
`,
        'urdf': {
          't.urdf': '<?xml version="1.0"?>'
        },
        'package.xml': '<?xml version="1.0"?>'
      }
    };

    mock(mockInit);


    const templateSource = path.join(process.cwd(), "test_source");

    const processCreateNode = new ProcessCreateNode(templateSource);

    const newPackageDir = path.join(process.cwd(), "test_package");

    // Create a new map of variables
    const variables = new Map<string, string>();

    await processCreateNode.createResourcePackage(newPackageDir, variables);
    let exists = fs.existsSync(path.join(newPackageDir, "urdf", "r.urdf"));
    assert.strictEqual(exists, false);

    // Reset for next test
    mock.restore();

    mock(mockInit);

    variables.set("include_urdf", "true");
    await processCreateNode.createResourcePackage(newPackageDir, variables);
    exists = fs.existsSync(path.join(newPackageDir, "urdf", "r.urdf"));
    assert.strictEqual(exists, true);
  });
});
