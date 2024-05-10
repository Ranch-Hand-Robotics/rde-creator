import * as assert from 'assert';

// You can import and use all API from the 'vscode' module
// as well as import your extension to test it
import * as vscode from 'vscode';
// import * as myExtension from '../../extension';
import {ProcessCreateNode} from '../../ProcessCreateNode';
import path from 'path';
import fs from 'fs';

suite('Extension Test Suite', () => {
	vscode.window.showInformationMessage('Start all tests.');

	test('Sample test', () => {
		assert.strictEqual(-1, [1, 2, 3].indexOf(5));
		assert.strictEqual(-1, [1, 2, 3].indexOf(0));
	});
});


// Create a new test suite for the ProcessCreateNode class
suite('ProcessCreateNode Test Suite', () => {
	// Test the createResourcePackage method
	test('createResourcePackage Test', () => {

		// Use mock-fs to mock the manifest.yaml file, a single cpp file, a single hpp file, a CMakeLists.txt file, and a package.xml file
		const mock = require('mock-fs');
		mock({
			'test_source': {
				'manifest.yaml': 'name: "template friendly name"\nversion: 0.0.0\ndescription: A templateMapping package\nmaintainers:\n  - name: templateMapping\n    email:\nlicense: "Apache 2.0"\nfile_mapping:\n  - TemplateNode.cpp: "{{variable1}}Node.cpp"\n  - TemplateNode.hpp: "{{variable2}}Node.hpp"\n  - CMakeLists.txt: CMakeLists.txt\n  - package.xml: package.xml',
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
		processCreateNode.createResourcePackage(newPackageDir, variables);
		// Check if the package directory was created
		assert.strictEqual(fs.existsSync(newPackageDir), true);
		// Check if the package directory contains the correct files
		assert.strictEqual(fs.existsSync(path.join(newPackageDir, "CMakeLists.txt")), true);
		assert.strictEqual(fs.existsSync(path.join(newPackageDir, "package.xml")), true);
		assert.strictEqual(fs.existsSync(path.join(newPackageDir, "testNode.cpp")), true);

		mock.restore();
	});
});
