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

    // Handle test case generation if test description is provided
    await this.handleTestGeneration(newPackageDir, variables, manifest);

    // Generate CONTRIBUTING.md with package documentation
    await this.generateContributingMd(newPackageDir, variables, manifest, false);
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
          
          // Get the new filename
          const key = filename as keyof typeof newMapping;
          
          // If this is a string mapping, use it directly
          if (newMapping[key] !== null && typeof newMapping[key] === 'string') {
            destinationPath = path.join(destination, newMapping[key]);
            extension.outputChannel?.appendLine(` Simple mapping ${filename} -> ${destinationPath}`);
          }
          
          // Check if the directory has a condition
          const conditionKey = "condition" as keyof typeof newMapping;
          if (newMapping[conditionKey]) {
            if (Array.isArray(newMapping[conditionKey])) {
              // Handle array of conditions according to specification
              const conditions = newMapping[conditionKey] as string[];
              condition = conditions.length > 0 ? conditions[0] : undefined;
              extension.outputChannel?.appendLine(` Found Conditions: ${conditions.join(', ')}`);
            } else {
              condition = newMapping[conditionKey] as string;
              extension.outputChannel?.appendLine(` Found Condition: ${condition}`);
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

  /**
   * Handles test case generation based on test description and template capabilities
   */
  private async handleTestGeneration(packageDir: string, variables: Map<string, string>, manifest: any) {
    const testDescription = variables.get("test_description");
    
    // Skip if no test description provided
    if (!testDescription || testDescription.trim() === "") {
      extension?.outputChannel?.appendLine("No test description provided, skipping enhanced test generation");
      return;
    }

    extension?.outputChannel?.appendLine(`Processing test description: ${testDescription}`);

    // Determine if we should generate custom tests or enhance existing ones
    const shouldGenerateCustomTests = this.shouldGenerateCustomTests(testDescription);
    
    if (shouldGenerateCustomTests) {
      await this.generateCustomTestCases(packageDir, variables, manifest, testDescription);
    } else {
      await this.enhanceExistingTests(packageDir, variables, testDescription);
    }
  }

  /**
   * Determines if custom test generation is needed based on test description content
   */
  private shouldGenerateCustomTests(testDescription: string): boolean {
    const customTestIndicators = [
      'custom test',
      'specific test',
      'additional test',
      'integration test',
      'end-to-end test',
      'performance test',
      'load test',
      'stress test',
      'edge case',
      'error handling test',
      'mock',
      'simulation'
    ];

    const lowerDescription = testDescription.toLowerCase();
    return customTestIndicators.some(indicator => lowerDescription.includes(indicator));
  }

  /**
   * Generates custom test cases based on the test description
   */
  private async generateCustomTestCases(packageDir: string, variables: Map<string, string>, manifest: any, testDescription: string) {
    extension?.outputChannel?.appendLine("Generating custom test cases based on description");
    
    // Determine template language/type from manifest or directory structure
    const templateType = await this.detectTemplateType(packageDir);
    
    // Generate test file content based on template type
    const customTestContent = await this.createCustomTestContent(templateType, variables, testDescription);
    
    if (customTestContent) {
      const testFileName = this.getCustomTestFileName(templateType, variables);
      const testFilePath = path.join(packageDir, 'test', testFileName);
      
      // Ensure test directory exists
      await fs.mkdir(path.dirname(testFilePath), { recursive: true });
      
      // Write custom test file
      await fs.writeFile(testFilePath, customTestContent);
      extension?.outputChannel?.appendLine(`Generated custom test file: ${testFilePath}`);
    }
  }

  /**
   * Enhances existing test files with test description information
   */
  private async enhanceExistingTests(packageDir: string, variables: Map<string, string>, testDescription: string) {
    extension?.outputChannel?.appendLine("Enhancing existing test files with description context");
    
    // Find existing test files
    const testDir = path.join(packageDir, 'test');
    
    try {
      const testFiles = await fs.readdir(testDir);
      
      for (const testFile of testFiles) {
        if (this.isTestFile(testFile)) {
          const testFilePath = path.join(testDir, testFile);
          await this.enhanceTestFile(testFilePath, variables, testDescription);
        }
      }
    } catch (error) {
      extension?.outputChannel?.appendLine(`No test directory found or error reading tests: ${error}`);
    }
  }

  /**
   * Detects the template type (python, cpp, nodejs) from the package structure
   */
  private async detectTemplateType(packageDir: string): Promise<string> {
    try {
      // Check for Python setup.py
      if (await this.fileExists(path.join(packageDir, 'setup.py'))) {
        return 'python';
      }
      
      // Check for C++ CMakeLists.txt
      if (await this.fileExists(path.join(packageDir, 'CMakeLists.txt'))) {
        return 'cpp';
      }
      
      // Check for Node.js package.json in subdirectory
      const files = await fs.readdir(packageDir);
      for (const file of files) {
        const subDir = path.join(packageDir, file);
        const stats = await fs.stat(subDir);
        if (stats.isDirectory() && await this.fileExists(path.join(subDir, 'package.json'))) {
          return 'nodejs';
        }
      }
    } catch (error) {
      extension?.outputChannel?.appendLine(`Error detecting template type: ${error}`);
    }
    
    return 'unknown';
  }

  /**
   * Creates custom test content based on template type and test description
   */
  private async createCustomTestContent(templateType: string, variables: Map<string, string>, testDescription: string): Promise<string | null> {
    const packageName = variables.get("package_name") || "my_package";
    const nodeName = variables.get("node_name") || "my_node";
    
    switch (templateType) {
      case 'python':
        return this.createPythonCustomTest(packageName, nodeName, testDescription, variables);
      case 'cpp':
        return this.createCppCustomTest(packageName, nodeName, testDescription, variables);
      case 'nodejs':
        return this.createNodejsCustomTest(packageName, nodeName, testDescription, variables);
      default:
        extension?.outputChannel?.appendLine(`Unknown template type: ${templateType}`);
        return null;
    }
  }

  /**
   * Creates Python custom test content
   */
  private createPythonCustomTest(packageName: string, nodeName: string, testDescription: string, variables: Map<string, string>): string {
    return `#!/usr/bin/env python3
"""
Custom test cases for ${packageName} based on user requirements.

Test Description: ${testDescription}

Author: ${variables.get("package_maintainer") || "Unknown"}
Date: ${variables.get("year") || new Date().getFullYear()}
"""

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading


class Custom${this.toPascalCase(nodeName)}Test(unittest.TestCase):
    """
    Custom test cases based on user-specified requirements:
    ${testDescription}
    """

    @classmethod
    def setUpClass(cls):
        """Set up test class."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Clean up test class."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        self.node = Node('custom_test_node')

    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()

    def test_custom_requirements(self):
        """Test custom requirements as specified by user."""
        # TODO: Implement custom test logic based on:
        # ${testDescription}
        
        # This is a template test that should be customized
        # based on the specific requirements above
        self.assertTrue(True, "Custom test template - implement based on requirements")

    def test_integration_scenario(self):
        """Test integration scenario derived from requirements."""
        # TODO: Implement integration test based on description
        
        from ${packageName}.node import ${this.toPascalCase(nodeName)}Node
        
        test_node = ${this.toPascalCase(nodeName)}Node()
        
        # Add custom integration test logic here
        # based on: ${testDescription}
        
        test_node.destroy_node()
        self.assertTrue(True, "Integration test template")

    def test_edge_cases(self):
        """Test edge cases mentioned in requirements."""
        # TODO: Implement edge case testing based on:
        # ${testDescription}
        
        self.assertTrue(True, "Edge case test template")


if __name__ == '__main__':
    unittest.main()
`;
  }

  /**
   * Creates C++ custom test content
   */
  private createCppCustomTest(packageName: string, nodeName: string, testDescription: string, variables: Map<string, string>): string {
    return `/**
 * @file test_custom_${packageName}.cpp
 * @brief Custom test cases for ${packageName} based on user requirements
 * 
 * Test Description: ${testDescription}
 * 
 * @author ${variables.get("package_maintainer") || "Unknown"}
 * @date ${variables.get("year") || new Date().getFullYear()}
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <thread>

#include "${packageName}/${packageName}_node.hpp"

using namespace std::chrono_literals;

/**
 * Custom test fixture for user-specified requirements:
 * ${testDescription}
 */
class Custom${this.toPascalCase(nodeName)}Test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<${packageName}::${this.toPascalCase(nodeName)}Node>();
    test_node_ = std::make_shared<rclcpp::Node>("custom_test_node");
  }

  void TearDown() override
  {
    node_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<${packageName}::${this.toPascalCase(nodeName)}Node> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
};

/**
 * @brief Test custom requirements as specified by user
 * 
 * Requirements: ${testDescription}
 */
TEST_F(Custom${this.toPascalCase(nodeName)}Test, CustomRequirements)
{
  // TODO: Implement custom test logic based on:
  // ${testDescription}
  
  ASSERT_NE(node_, nullptr);
  
  // This is a template test that should be customized
  // based on the specific requirements above
  SUCCEED() << "Custom test template - implement based on requirements";
}

/**
 * @brief Test integration scenario derived from requirements
 */
TEST_F(Custom${this.toPascalCase(nodeName)}Test, IntegrationScenario)
{
  // TODO: Implement integration test based on description:
  // ${testDescription}
  
  // Add custom integration test logic here
  SUCCEED() << "Integration test template";
}

/**
 * @brief Test edge cases mentioned in requirements
 */
TEST_F(Custom${this.toPascalCase(nodeName)}Test, EdgeCases)
{
  // TODO: Implement edge case testing based on:
  // ${testDescription}
  
  SUCCEED() << "Edge case test template";
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
`;
  }

  /**
   * Creates Node.js custom test content
   */
  private createNodejsCustomTest(packageName: string, nodeName: string, testDescription: string, variables: Map<string, string>): string {
    return `/**
 * @file test/custom.test.js
 * @brief Custom test cases for ${packageName} based on user requirements
 * 
 * Test Description: ${testDescription}
 * 
 * @author ${variables.get("package_maintainer") || "Unknown"}
 * @date ${variables.get("year") || new Date().getFullYear()}
 */

const rclnodejs = require('rclnodejs');
const ${this.toPascalCase(nodeName)}Node = require('../${packageName}/node');

/**
 * Custom test suite based on user-specified requirements:
 * ${testDescription}
 */
describe('Custom ${this.toPascalCase(nodeName)}Node Tests', () => {
  let node;
  let testNode;

  beforeAll(async () => {
    await rclnodejs.init();
  });

  afterAll(async () => {
    await rclnodejs.shutdown();
  });

  beforeEach(async () => {
    node = new ${this.toPascalCase(nodeName)}Node();
    testNode = rclnodejs.createNode('custom_test_node');
    await node.init();
  });

  afterEach(async () => {
    if (node) {
      await node.cleanup();
    }
    if (testNode) {
      testNode.destroy();
    }
  });

  /**
   * Test custom requirements as specified by user
   * Requirements: ${testDescription}
   */
  test('should meet custom requirements', async () => {
    // TODO: Implement custom test logic based on:
    // ${testDescription}
    
    expect(node).toBeDefined();
    
    // This is a template test that should be customized
    // based on the specific requirements above
    expect(true).toBe(true); // Custom test template
  });

  /**
   * Test integration scenario derived from requirements
   */
  test('should pass integration scenario', async () => {
    // TODO: Implement integration test based on description:
    // ${testDescription}
    
    // Add custom integration test logic here
    expect(true).toBe(true); // Integration test template
  });

  /**
   * Test edge cases mentioned in requirements
   */
  test('should handle edge cases', async () => {
    // TODO: Implement edge case testing based on:
    // ${testDescription}
    
    expect(true).toBe(true); // Edge case test template
  });
});
`;
  }

  /**
   * Gets the appropriate custom test file name based on template type
   */
  private getCustomTestFileName(templateType: string, variables: Map<string, string>): string {
    const packageName = variables.get("package_name") || "my_package";
    
    switch (templateType) {
      case 'python':
        return `test_custom_${packageName}.py`;
      case 'cpp':
        return `test_custom_${packageName}.cpp`;
      case 'nodejs':
        return 'custom.test.js';
      default:
        return `test_custom_${packageName}.txt`;
    }
  }

  /**
   * Enhances an existing test file with test description context
   */
  private async enhanceTestFile(testFilePath: string, variables: Map<string, string>, testDescription: string) {
    try {
      let content = await fs.readFile(testFilePath, 'utf-8');
      
      // Add test description as a comment at the top of existing test files
      const testDescriptionComment = this.createTestDescriptionComment(testFilePath, testDescription);
      
      // Insert after the first line (usually shebang or file comment)
      const lines = content.split('\n');
      if (lines.length > 0) {
        lines.splice(1, 0, testDescriptionComment);
        content = lines.join('\n');
        
        await fs.writeFile(testFilePath, content);
        extension?.outputChannel?.appendLine(`Enhanced test file: ${testFilePath}`);
      }
    } catch (error) {
      extension?.outputChannel?.appendLine(`Error enhancing test file ${testFilePath}: ${error}`);
    }
  }

  /**
   * Creates appropriate test description comment based on file type
   */
  private createTestDescriptionComment(testFilePath: string, testDescription: string): string {
    const ext = path.extname(testFilePath);
    
    switch (ext) {
      case '.py':
        return `"""
Test Requirements: ${testDescription}
"""`;
      case '.cpp':
      case '.hpp':
        return `/**
 * Test Requirements: ${testDescription}
 */`;
      case '.js':
        return `/**
 * Test Requirements: ${testDescription}
 */`;
      default:
        return `# Test Requirements: ${testDescription}`;
    }
  }

  /**
   * Utility functions
   */
  private async fileExists(filePath: string): Promise<boolean> {
    try {
      await fs.access(filePath);
      return true;
    } catch {
      return false;
    }
  }

  private isTestFile(fileName: string): boolean {
    return fileName.includes('test') && 
           (fileName.endsWith('.py') || fileName.endsWith('.cpp') || fileName.endsWith('.js'));
  }

  private toPascalCase(str: string): string {
    return str.replace(/(?:^|[-_])(\w)/g, (_, c) => c.toUpperCase());
  }

  /**
   * Generates CONTRIBUTING.md file with package documentation
   */
  public async generateContributingMd(
    packageDir: string,
    variables: Map<string, string>,
    manifest: any,
    isAiGenerated: boolean = false,
    aiPrompt?: string,
    naturalLanguageDescription?: string,
    testDescription?: string
  ) {
    try {
      extension?.outputChannel?.appendLine("Generating CONTRIBUTING.md documentation");

      // Load the CONTRIBUTING.md template
      const contributingTemplatePath = path.join(__dirname, '..', 'templates', 'CONTRIBUTING.md.hbs');
      let contributingTemplate: string;

      try {
        contributingTemplate = await fs.readFile(contributingTemplatePath, 'utf-8');
      } catch (error) {
        extension?.outputChannel?.appendLine(`CONTRIBUTING.md template not found at ${contributingTemplatePath}, using fallback`);
        contributingTemplate = this.getFallbackContributingTemplate();
      }

      // Prepare template variables
      const templateVars = this.prepareContributingVariables(
        variables,
        manifest,
        isAiGenerated,
        aiPrompt,
        naturalLanguageDescription,
        testDescription
      );

      // Compile and render template
      const template = Handlebars.compile(contributingTemplate);
      const renderedContent = template(templateVars);

      // Write CONTRIBUTING.md file
      const contributingPath = path.join(packageDir, 'CONTRIBUTING.md');
      await fs.writeFile(contributingPath, renderedContent);
      
      extension?.outputChannel?.appendLine(`Generated CONTRIBUTING.md at ${contributingPath}`);
    } catch (error) {
      extension?.outputChannel?.appendLine(`Error generating CONTRIBUTING.md: ${error}`);
    }
  }

  /**
   * Prepares variables for CONTRIBUTING.md template
   */
  private prepareContributingVariables(
    variables: Map<string, string>,
    manifest: any,
    isAiGenerated: boolean,
    aiPrompt?: string,
    naturalLanguageDescription?: string,
    testDescription?: string
  ): any {
    const templateVars = Object.fromEntries(variables);

    // Add generation metadata
    templateVars.ai_generated = isAiGenerated;
    templateVars.generation_date = new Date().toISOString().split('T')[0];
    templateVars.template_name = manifest.name;
    templateVars.ai_directive = manifest.ai_directive || '';

    // Add AI-specific information
    if (isAiGenerated) {
      templateVars.natural_language_description = naturalLanguageDescription || '';
      templateVars.ai_prompt_used = aiPrompt || '';
      templateVars.generation_parameters = this.extractGenerationParameters(variables);
    } else {
      templateVars.template_parameters = this.extractTemplateParameters(variables);
    }

    // Add test description if provided
    if (testDescription && testDescription.trim()) {
      templateVars.test_description = testDescription.trim();
    }

    // Detect template language
    templateVars.template_language_python = this.detectTemplateLanguage(variables, manifest) === 'python';
    templateVars.template_language_cpp = this.detectTemplateLanguage(variables, manifest) === 'cpp';
    templateVars.template_language_nodejs = this.detectTemplateLanguage(variables, manifest) === 'nodejs';

    // Check for custom tests
    templateVars.has_custom_tests = this.shouldGenerateCustomTests(testDescription || '');

    // ROS 2 distribution
    templateVars.ros_distro = 'humble'; // Default, could be made configurable

    return templateVars;
  }

  /**
   * Extracts generation parameters for AI-generated packages
   */
  private extractGenerationParameters(variables: Map<string, string>): Record<string, string> {
    const params: Record<string, string> = {};
    
    // Filter out internal variables and include user-relevant ones
    const relevantKeys = [
      'package_name', 'package_description', 'package_version', 
      'package_maintainer', 'package_license', 'node_name',
      'topic_name', 'service_name', 'timer_period'
    ];

    for (const [key, value] of variables.entries()) {
      if (relevantKeys.includes(key)) {
        params[key] = value;
      }
    }

    return params;
  }

  /**
   * Extracts template parameters for template-generated packages
   */
  private extractTemplateParameters(variables: Map<string, string>): Record<string, string> {
    const params: Record<string, string> = {};
    
    for (const [key, value] of variables.entries()) {
      if (!key.endsWith('_file') && key !== 'year' && key !== 'test_description') {
        params[key] = value;
      }
    }

    return params;
  }

  /**
   * Detects template language from variables and manifest
   */
  private detectTemplateLanguage(variables: Map<string, string>, manifest: any): string {
    const templateName = manifest.name?.toLowerCase() || '';
    
    if (templateName.includes('python')) {
      return 'python';
    }
    if (templateName.includes('c++') || templateName.includes('cpp')) {
      return 'cpp';
    }
    if (templateName.includes('node') && templateName.includes('js')) {
      return 'nodejs';
    }
    
    // Fallback detection based on package structure
    return 'python'; // Default fallback
  }

  /**
   * Provides a fallback CONTRIBUTING.md template if the main template file is not found
   */
  private getFallbackContributingTemplate(): string {
    return `# Contributing to {{package_name}}

## Package Overview

**Package Name:** {{package_name}}  
**Description:** {{package_description}}  
**Version:** {{package_version}}  
**License:** {{package_license}}  
**Maintainer:** {{package_maintainer}}

{{#if ai_generated}}
## AI Generation Details

This package was generated using AI with the following requirements:

**User Requirements:**
\`\`\`
{{natural_language_description}}
\`\`\`

{{#if test_description}}
**Test Requirements:**
\`\`\`
{{test_description}}
\`\`\`
{{/if}}
{{else}}
## Template Generation

This package was generated from the {{template_name}} template.
{{/if}}

## Architecture Overview

\`\`\`mermaid
graph TB
    subgraph "{{package_name}} Package"
        NODE[{{node_name}}]
        {{#if include_publisher}}
        NODE --> |publishes| TOPIC[{{topic_name}}]
        {{/if}}
        {{#if include_subscriber}}
        TOPIC_IN[{{topic_name}}] --> |subscribes| NODE
        {{/if}}
        {{#if include_service}}
        CLIENT --> |request| NODE
        NODE --> |response| CLIENT
        {{/if}}
    end
\`\`\`

## Development

### Building
\`\`\`bash
cd ~/ros2_ws
colcon build --packages-select {{package_name}}
source install/setup.bash
\`\`\`

### Running
\`\`\`bash
ros2 run {{package_name}} {{node_name}}
\`\`\`

### Testing
\`\`\`bash
colcon test --packages-select {{package_name}}
\`\`\`

---
*Generated on {{generation_date}}*
`;
  }
}
