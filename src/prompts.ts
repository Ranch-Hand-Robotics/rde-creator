// Copyright (c) Ranch Hand Robotics, LLC. All rights reserved.
// Licensed under the MIT License.

/**
 * Constructs an AI prompt for ROS 2 package generation based on template and user requirements
 */
// The original single-shot prompt is replaced by staged prompt builders
// to enable a sub-agent, multi-step generation workflow that reduces response size
export function constructPlanPrompt(
  templateContent: string,
  manifest: any,
  variablesObj: Record<string, string>,
  naturalLanguageDescription: string,
  maxResponseSize: number
): string {
  const aiDirective = manifest.ai_directive || '';
  return `You are an expert ROS 2 developer. Produce a JSON plan for generating the package in multiple steps.

IMPORTANT: Return ONLY raw JSON. Do NOT wrap in markdown code blocks. Do NOT use \`\`\`json formatting.
Your response must start with { and end with }.

Example output format:
{
  "files":[{"path":"package.xml","estimate_bytes":200}],
  "chunks":{"large_file.cpp":3}
}

Requirements:
- List all files to generate under "files" as objects: {"path":"...","estimate_bytes":123}
- Provide a "chunks" map: file path -> number of chunks (<=10) to split large files
- Use ${maxResponseSize} as an upper bound for any single response size
${aiDirective ? `
TEMPLATE AI DIRECTIVE:
${aiDirective}
` : ''}
User variables: ${JSON.stringify(variablesObj)}
User request: ${naturalLanguageDescription}
`;
}

export function constructFileChunkPrompt(
  templateContent: string,
  manifest: any,
  variablesObj: Record<string, string>,
  naturalLanguageDescription: string,
  filePath: string,
  chunkIndex: number,
  totalChunks: number
): string {
  const aiDirective = manifest.ai_directive || '';
  return `You are an expert ROS 2 developer. Return JSON for chunk ${chunkIndex}/${totalChunks} of file: ${filePath}

IMPORTANT: Return ONLY raw JSON. Do NOT wrap in markdown code blocks. Do NOT use \`\`\`json formatting.
Your response must start with { and end with }.

Example output format:
{
  "file":"${filePath}",
  "chunk_index":${chunkIndex},
  "total_chunks":${totalChunks},
  "content":"file content here with \\n for newlines"
}

Requirements:
- Content must be a JSON string with special characters escaped
- Use \\n for newlines, escape quotes as \\" and backslashes as \\\\
- chunk_index is 1-based
- Concatenation of all chunks in order must equal the full file content
- Generate ONLY this chunk (${chunkIndex}/${totalChunks}), not the entire file
${aiDirective ? `
TEMPLATE AI DIRECTIVE:
${aiDirective}
` : ''}
User variables: ${JSON.stringify(variablesObj)}
User request: ${naturalLanguageDescription}
`;
}

export function constructFollowupPromptInstructions(maxResponseSize: number): string {
  return `

CRITICAL FORMATTING RULES:
- Return raw JSON ONLY (no markdown, no code blocks, no \`\`\`json)
- Response must start with { and end with }
- Maximum ${maxResponseSize} characters per response
- Escape all special characters properly (\\n for newlines, \\" for quotes)
- For plans: include "files" array and "chunks" object
- For chunks: include file, chunk_index, total_chunks, content fields`;
}