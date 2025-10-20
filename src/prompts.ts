// Copyright (c) Ranch Hand Robotics, LLC. All rights reserved.
// Licensed under the MIT License.

/**
 * Constructs an AI prompt for ROS 2 package generation based on template and user requirements
 */
export function constructAIPrompt(
  templateContent: string,
  manifest: any,
  variables: Map<string, string>,
  naturalLanguageDescription: string,
  testDescription?: string
): string {
  const variablesObj = Object.fromEntries(variables);

  // Extract AI directive from manifest if present
  const aiDirective = manifest.ai_directive || '';

  return `
You are an expert ROS 2 developer tasked with generating a complete ROS 2 package based on a template and user requirements.

${aiDirective ? `## TEMPLATE-SPECIFIC AI DIRECTIVE (HIGH PRIORITY - FOLLOW EXACTLY)
${aiDirective}

CRITICAL: Follow the directory structure and file organization specified in the AI directive exactly.
CRITICAL: Only include test files if explicitly requested by the user in the test requirements section.
CRITICAL: Do not include any files not mentioned in the AI directive unless specifically requested.
CRITICAL: Respond with VALID JSON only. Your response must be parseable as JSON and contain only the package structure.
CRITICAL: Package.xml must be included in the root of the package and must have the following fields correctly filled out; if not supplied provide a best guess based on the user's natural language description:
- name
- version
- description
- maintainer
- license
- author
- buildtool_depend
- depend
- test_depend (if tests are requested)
- export


` : ''}## Template Information
The following template files are available for reference:
${templateContent}

## Package Manifest (Metadata - Do not include manifest.yaml in generated package)
${JSON.stringify(manifest, null, 2)}

## Important: File vs Directory Interpretation
When reading the manifest structure:
- Lines ending with '/' indicate directories (e.g., 'launch/', 'urdf/')
- Lines with file extensions indicate files (e.g., 'package.xml', 'setup.py')  
- Lines without extensions but with specific names indicate files (e.g., 'resource/package_name' is a FILE)
- The 'resource/package_name' entry MUST be an empty file, not a directory
- Do NOT create directories where files are specified

## User Parameters (Replace {{variable}} placeholders with these values)
${JSON.stringify(variablesObj, null, 2)}

## User Requirements (CRITICAL - Implement exactly as specified)
${naturalLanguageDescription}

**IMPORTANT**: The user's natural language description above is the PRIMARY requirement.
Analyze it carefully and implement EXACTLY what is requested. Do not add extra features or change the functionality unless explicitly asked.
If the description is unclear, make reasonable assumptions but document them in comments.

## Test Requirements
${testDescription && testDescription.trim() ? `
The user has specified the following test case requirements:
${testDescription}

Please generate comprehensive test cases that:
- Test the functionality described above
- Cover edge cases and error scenarios mentioned
- Follow ROS 2 testing best practices
- Include unit tests, integration tests, and any specific test types mentioned
- Use appropriate test frameworks (gtest for C++, unittest/pytest for Python, Jest for Node.js)
- Include proper setup and teardown procedures
- Test ROS 2 specific functionality (publishers, subscribers, services, parameters)
- Validate message flows and service calls
- Include performance and reliability tests where mentioned

` : 'IMPORTANT: No tests were requested by the user. Do NOT include any test files or test directories in the generated package.'}

## Instructions
1. **CRITICAL**: Follow the exact directory structure and file layout specified in the AI directive
2. **CRITICAL**: Create files as needed to support the request
3. **CRITICAL**: Do not include manifest.yaml or any metadata files in the generated package
4. Analyze the user's natural language description and adapt the package accordingly
5. Use the provided parameters to customize the package (replace ALL {{variable}} placeholders)
6. Generate a complete, functional ROS 2 package that meets the user's requirements
7. Ensure all generated files follow ROS 2 best practices and conventions
8. Include appropriate dependencies, launch files, and configuration as needed
9. Make the package production-ready with proper error handling and documentation
10. Use modern ROS 2 patterns (rclpy for Python, rclcpp for C++, rclnodejs for Node.js, rclrust for Rust)
11. Include proper QoS settings where appropriate
12. Add comprehensive logging and error handling
13. Follow ROS 2 naming conventions and message standards
14. Include launch files with proper parameter handling
15. ${testDescription && testDescription.trim() ? 'Generate comprehensive test suites based on test requirements' : '**DO NOT generate any test files** - no tests were requested'}
16. Ensure test files follow language-specific testing conventions and ROS 2 patterns
17. **IMPORTANT:** Generate a comprehensive CONTRIBUTING.md file that documents:
    - Complete package architecture with Mermaid diagrams showing:
      * Node architecture and communication patterns
      * Data flow between components
      * Service/action interfaces
      * Parameter dependencies
      * Launch file structure
    - Detailed explanation of each node's purpose and responsibilities
    - Communication patterns (publishers, subscribers, services, actions)
    - Parameter documentation with defaults and valid ranges
    - Build and installation instructions
    - Testing strategy and how to run tests
    - Development guidelines and code organization
    - Dependencies and their purposes
    - Troubleshooting common issues
    - The original user requirements that generated this package
    - AI generation metadata (date, model, user prompt summary)

    AI:
  - Agents.md must be included in the package root with instructions for future AI agents that may interact with this package.


## Important ROS 2 Requirements
- Include spin() or equivalent for event processing
- Use appropriate message types from standard ROS 2 packages
- Include proper exception handling
- Add docstrings and comments for maintainability
- Do not include any placeholder text like "TODO" or "FIXME".
- If you are confused about any requirement, make a reasonable assumption and proceed, but add a comment explaining your confusion and decisions used to move forward.

Generate the ROS 2 package now:
## Output Format (JSON only)
{
  "directories": ["dir1", "dir2"],
  "files": {
    "path/to/file.py": "file content here",
    "package.xml": "<?xml version='1.0'?>..."
  }
}
`;
}