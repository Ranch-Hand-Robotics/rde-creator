// Copyright (c) Ranch Hand Robotics, LLC. All rights reserved.
// Licensed under the MIT License.

import React, { useState, useEffect } from 'react';
import { createRoot } from 'react-dom/client';

// VS Code API
declare const acquireVsCodeApi: () => any;
const vscode = acquireVsCodeApi();

interface WebviewMessage {
  command: string;
  text?: string;
  type?: string;
  variables?: Record<string, string>;
  naturalLanguageDescription?: string;
  testDescription?: string;
  manifests?: string;
  selectedModel?: string;
  selectedTestModel?: string;
  files?: string[];
}

interface PackageManifest {
  name: string;
  description: string;
  icon?: string;
  short_description?: string;
  directory: string;
}

interface LanguageModel {
  id: string;
  name: string;
  vendor: string;
  family: string;
  displayName: string;
}

const App: React.FC = () => {
  const [packageName, setPackageName] = useState('');
  const [packageMaintainer, setPackageMaintainer] = useState('');
  const [packageVersion, setPackageVersion] = useState('0.0.1');
  const [packageDescription, setPackageDescription] = useState('');
  const [packageLicense, setPackageLicense] = useState('MIT');
  const [packageType, setPackageType] = useState<string>('');
  const [availableManifests, setAvailableManifests] = useState<PackageManifest[]>([]);
  const [naturalLanguageDescription, setNaturalLanguageDescription] = useState('');
  const [testDescription, setTestDescription] = useState('');

  const [generationProgress, setGenerationProgress] = useState<string[]>([]);
  const [isGenerating, setIsGenerating] = useState(false);
  const [isLoadingManifests, setIsLoadingManifests] = useState(true);
  const [generationComplete, setGenerationComplete] = useState(false);
  const [generationStopped, setGenerationStopped] = useState(false);

  const [availableModels, setAvailableModels] = useState<LanguageModel[]>([]);
  const [selectedModel, setSelectedModel] = useState<string>('');
  const [selectedTestModel, setSelectedTestModel] = useState<string>('');
  const [generateTests, setGenerateTests] = useState<boolean>(false);
  
  // Track generation plan and progress
  const [planSteps, setPlanSteps] = useState<Array<{file: string, completed: boolean, generating: boolean}>>([]);
  const [currentStep, setCurrentStep] = useState<string>('');

  useEffect(() => {
    // Request manifests on load
    vscode.postMessage({ command: 'webviewLoaded' });

    const messageHandler = (event: MessageEvent) => {
      const message = event.data;
      switch (message.command) {
        case 'setManifests':
          try {
            const manifestsData = JSON.parse(message.manifests);
            const manifests: PackageManifest[] = Object.entries(manifestsData).map(([dir, data]: [string, any]) => ({
              name: data.name || dir,
              description: data.description || '',
              icon: data.icon || '📦',
              short_description: data.short_description || data.description || '',
              directory: dir
            }));
            setAvailableManifests(manifests);
            // Set default to first manifest if available (prefer python_node if exists)
            if (manifests.length > 0) {
              const pythonManifest = manifests.find(m => m.directory === 'python_node');
              const defaultManifest = pythonManifest || manifests[0];
              setPackageType(defaultManifest.directory);
            }
            setIsLoadingManifests(false);
          } catch (error) {
            console.error('Error parsing manifests:', error);
            setIsLoadingManifests(false);
          }
          break;
        case 'setAvailableModels':
          try {
            const models: LanguageModel[] = message.models;
            const lastSelectedModel = message.lastSelectedModel || 'auto';
            
            // Add "Auto" option at the beginning
            const modelsWithAuto = [
              { id: 'auto', name: 'Auto', vendor: 'auto', family: 'auto', displayName: 'Auto (use preferred model)' },
              ...models
            ];
            
            setAvailableModels(modelsWithAuto);
            
            // Set default based on last selected model
            if (lastSelectedModel === 'auto' || !modelsWithAuto.find(m => m.id === lastSelectedModel)) {
              setSelectedModel('auto');
            } else {
              setSelectedModel(lastSelectedModel);
            }
            
            // Set default test model (prefer different from main model)
            const lastSelectedTestModel = message.lastSelectedTestModel || 'auto';
            if (lastSelectedTestModel === 'auto' || !modelsWithAuto.find(m => m.id === lastSelectedTestModel)) {
              setSelectedTestModel('auto');
            } else {
              setSelectedTestModel(lastSelectedTestModel);
            }
          } catch (error) {
            console.error('Error parsing models:', error);
          }
          break;
        case 'aiPlan':
          // Receive the full file list from the AI plan
          if (message.files && Array.isArray(message.files)) {
            setPlanSteps(message.files.map((file: string) => ({ file, completed: false, generating: false })));
            setCurrentStep(`Plan received: ${message.files.length} files to generate`);
          }
          break;
        case 'aiTestPlan':
          // Receive the test file list and add to plan
          if (message.files && Array.isArray(message.files)) {
            setPlanSteps(prev => [
              ...prev,
              ...message.files.map((file: string) => ({ file, completed: false, generating: false }))
            ]);
            setCurrentStep(`Test plan received: ${message.files.length} test files to generate`);
          }
          break;
        case 'aiProgress':
          setGenerationProgress(prev => [...prev, message.text]);
          
          // Parse progress messages to update plan steps
          const text = message.text;
          
          // Detect plan received - extract file list
          if (text.includes('Plan received from AI')) {
            // Next messages should contain file information
            setCurrentStep('Received generation plan');
          } else if (text.match(/Requesting chunk \d+\/\d+ for (.+)/)) {
            const match = text.match(/Requesting chunk \d+\/\d+ for (.+)/);
            if (match) {
              const filePath = match[1];
              setCurrentStep(`Generating ${filePath}`);
              // Mark file as generating, add if not already there
              setPlanSteps(prev => {
                const existing = prev.find(s => s.file === filePath);
                if (!existing) {
                  return [...prev, { file: filePath, completed: false, generating: true }];
                }
                // Mark as generating if not already completed
                return prev.map(s => 
                  s.file === filePath && !s.completed ? { ...s, generating: true } : s
                );
              });
            }
          } else if (text.match(/Completed file: (.+)/)) {
            const match = text.match(/Completed file: (.+)/);
            if (match) {
              const filePath = match[1];
              setCurrentStep(`Completed ${filePath}`);
              // Mark as completed and not generating
              setPlanSteps(prev => prev.map(s => 
                s.file === filePath ? { ...s, completed: true, generating: false } : s
              ));
            }
          } else if (text.includes('Sending plan request')) {
            setCurrentStep('Requesting generation plan...');
          } else if (text.includes('Template files loaded')) {
            setCurrentStep('Analyzing template structure');
          } else if (text.includes('Starting test generation')) {
            setCurrentStep('Starting test generation');
          } else if (text.match(/Requesting test chunk \d+\/\d+ for (.+)/)) {
            const match = text.match(/Requesting test chunk \d+\/\d+ for (.+)/);
            if (match) {
              const filePath = match[1];
              setCurrentStep(`Generating test: ${filePath}`);
              // Mark test file as generating, add if not already there
              setPlanSteps(prev => {
                const existing = prev.find(s => s.file === filePath);
                if (!existing) {
                  return [...prev, { file: filePath, completed: false, generating: true }];
                }
                return prev.map(s => 
                  s.file === filePath && !s.completed ? { ...s, generating: true } : s
                );
              });
            }
          } else if (text.match(/Completed test file: (.+)/)) {
            const match = text.match(/Completed test file: (.+)/);
            if (match) {
              const filePath = match[1];
              setCurrentStep(`Completed test: ${filePath}`);
              setPlanSteps(prev => prev.map(s => 
                s.file === filePath ? { ...s, completed: true, generating: false } : s
              ));
            }
          }
          break;
        case 'aiComplete':
          setIsGenerating(false);
          setGenerationComplete(true);
          setCurrentStep('Generation complete');
          break;
      }
    };

    window.addEventListener('message', messageHandler);
    return () => window.removeEventListener('message', messageHandler);
  }, []);

  const handleCreatePackage = () => {
    const finalPackageName = packageName.trim();
    const finalPackageMaintainer = packageMaintainer.trim() || "your.email@example.com";
    const finalPackageVersion = packageVersion.trim() || "0.0.1";
    const finalPackageDescription = packageDescription.trim();
    const finalPackageLicense = packageLicense.trim() || "MIT";
    const finalNaturalLanguageDescription = naturalLanguageDescription.trim();

    // Validation
    if (!finalPackageName) {
      vscode.postMessage({ command: 'error', text: 'Package name is required' });
      return;
    }

    if (!finalNaturalLanguageDescription) {
      vscode.postMessage({ command: 'error', text: 'Please describe what you want your ROS 2 package to do' });
      return;
    }

    if (!packageType) {
      vscode.postMessage({ command: 'error', text: 'Please select a package type' });
      return;
    }

    if (!selectedModel) {
      vscode.postMessage({ command: 'error', text: 'Please select an AI model' });
      return;
    }

    const message: WebviewMessage = {
      command: 'createPackageWithAI',
      type: packageType,
      variables: {
        package_name: finalPackageName,
        package_maintainer: finalPackageMaintainer,
        package_version: finalPackageVersion,
        package_description: finalPackageDescription,
        package_license: finalPackageLicense
      },
      naturalLanguageDescription: finalNaturalLanguageDescription,
      testDescription: generateTests ? testDescription.trim() : '',
      selectedModel: selectedModel,
      selectedTestModel: generateTests ? selectedTestModel : undefined
    };

    setIsGenerating(true);
    setGenerationProgress([]);
    setGenerationComplete(false);
    setGenerationStopped(false);
    setPlanSteps([]);
    setCurrentStep('Starting generation...');
    vscode.postMessage(message);
  };

  const renderCreatePackagePage = () => (
    <div id="create_package_page">
      <div className="header-bar">
        <h1>🤖 AI-Powered ROS 2 Package Generator</h1>
        <p className="subtitle">Describe your package and let AI bootstrap ROS 2 packages</p>
      </div>
      
      <div className="main-content">
        {/* Package Type Selection */}
        <div className="component-container">
          <h2>📦 Package Type</h2>
          {isLoadingManifests ? (
            <div className="loading-manifests">
              <div className="spinner"></div>
              <p>Loading available package types...</p>
            </div>
          ) : availableManifests.length === 0 ? (
            <div className="error-message">
              <p>❌ No package templates found. Please check your installation.</p>
            </div>
          ) : (
            <div className="package-type-selector">
              {availableManifests.map((manifest) => (
                <button 
                  key={manifest.directory}
                  className={`type-button ${packageType === manifest.directory ? 'active' : ''}`}
                  onClick={() => setPackageType(manifest.directory)}
                  title={manifest.description}
                >
                  <div className="type-icon">{manifest.icon}</div>
                  <div className="type-label">{manifest.name}</div>
                  <div className="type-description">{manifest.short_description}</div>
                </button>
              ))}
            </div>
          )}
        </div>

        {/* Package Description */}
        <div className="component-container">
          <h2>✨ What should your package do?</h2>
          <div className="form-field">
            <textarea
              id="naturalLanguageDescription"
              placeholder="Example: Create a publisher node that reads IMU data from a SparkFun 9DoF sensor (ISM330DHCX, MMC5983MA) at 100Hz using i2c. Include a calibration service that can be triggered to update sensor offsets, and publish both raw and calibrated data on separate topics. Add parameter support for configuring the sample rate and i2c bus address."
              rows={6}
              value={naturalLanguageDescription}
              onChange={(e) => setNaturalLanguageDescription(e.target.value)}
              className="text-area large"
              required
            />
            <small className="help-text">
              💡 Be specific! Include details about topics, services, parameters, sensors, algorithms, and behaviors.
            </small>
          </div>
          
          <div className="form-field">
            <label htmlFor="modelSelect">AI Model for Code Generation</label>
            <select
              id="modelSelect"
              value={selectedModel}
              onChange={(e) => setSelectedModel(e.target.value)}
              className="select-field"
              disabled={availableModels.length === 0}
            >
              {availableModels.length === 0 ? (
                <option value="">Loading models...</option>
              ) : (
                availableModels.map((model) => (
                  <option key={model.id} value={model.id}>
                    {model.displayName}
                  </option>
                ))
              )}
            </select>
            <small className="help-text">
              💡 Select the AI model to use for generating your ROS 2 package code. Different models may produce different results.
            </small>
          </div>
        </div>

        {/* Test Generation */}
        <div className="component-container">
          <h2>🧪 Test Generation (Optional)</h2>
          <div className="form-field">
            <label className="checkbox-label">
              <input
                type="checkbox"
                checked={generateTests}
                onChange={(e) => setGenerateTests(e.target.checked)}
              />
              <span>Generate tests for this package</span>
            </label>
          </div>
          
          {generateTests && (
            <>
              <div className="form-field">
                <label htmlFor="testDescription">Test Requirements</label>
                <textarea
                  id="testDescription"
                  placeholder="Example: Create unit tests for the IMU data processing, verify calibration service functionality, test error handling for disconnected sensors, and validate parameter updates."
                  rows={3}
                  value={testDescription}
                  onChange={(e) => setTestDescription(e.target.value)}
                  className="text-area"
                />
                <small className="help-text">
                  Describe what tests should be generated. Be specific about test scenarios and edge cases.
                </small>
              </div>
              
              <div className="form-field">
                <label htmlFor="testModelSelect">AI Model for Test Generation</label>
                <select
                  id="testModelSelect"
                  value={selectedTestModel}
                  onChange={(e) => setSelectedTestModel(e.target.value)}
                  className="select-field"
                  disabled={availableModels.length === 0}
                >
                  {availableModels.length === 0 ? (
                    <option value="">Loading models...</option>
                  ) : (
                    availableModels.map((model) => (
                      <option key={model.id} value={model.id}>
                        {model.displayName}
                      </option>
                    ))
                  )}
                </select>
                <small className="help-text">
                  💡 Recommended: Use a different model than the one generating code for better test coverage and different perspectives on edge cases.
                </small>
              </div>
            </>
          )}
        </div>

        {/* Package Metadata */}
        <div className="component-container">
          <h2>📋 Package Metadata</h2>
          <div className="form-grid">
            <div className="form-field">
              <label htmlFor="package_name">Package Name *</label>
              <input
                id="package_name"
                type="text"
                placeholder="my_awesome_package"
                value={packageName}
                onChange={(e) => setPackageName(e.target.value)}
                className="text-field"
                required
              />
            </div>
            <div className="form-field">
              <label htmlFor="package_version">Version *</label>
              <input
                id="package_version"
                type="text"
                placeholder="0.0.1"
                value={packageVersion}
                onChange={(e) => setPackageVersion(e.target.value)}
                className="text-field"
              />
            </div>
            <div className="form-field">
              <label htmlFor="package_maintainer">Maintainer Email *</label>
              <input
                id="package_maintainer"
                type="email"
                placeholder="your.email@example.com"
                value={packageMaintainer}
                onChange={(e) => setPackageMaintainer(e.target.value)}
                className="text-field"
              />
            </div>
            <div className="form-field">
              <label htmlFor="package_license">License</label>
              <input
                id="package_license"
                type="text"
                placeholder="MIT"
                value={packageLicense}
                onChange={(e) => setPackageLicense(e.target.value)}
                className="text-field"
              />
            </div>
          </div>
          <div className="form-field">
            <label htmlFor="package_description">Description</label>
            <textarea
              id="package_description"
              placeholder="A brief description for your package."
              rows={2}
              value={packageDescription}
              onChange={(e) => setPackageDescription(e.target.value)}
              className="text-area"
            />
          </div>
        </div>
      </div>
      
      <div className="button-bar">
        <button 
          className="button secondary" 
          onClick={() => vscode.postMessage({ command: 'cancel' })}
        >
          Cancel
        </button>
        <button 
          id="create_package_button" 
          onClick={handleCreatePackage} 
          className="button primary"
          disabled={!packageName.trim() || !naturalLanguageDescription.trim() || !packageType || !selectedModel || isLoadingManifests}
        >
          🚀 Generate Package with AI
        </button>
      </div>
    </div>
  );

  const renderGeneratingPage = () => (
    <div id="generating_page">
      <div className="header-bar">
        <h1>🤖 Generating ROS 2 Package with AI</h1>
      </div>
      <div className="component-row">
        <div className="component-container">
          <h2>AI Generation Progress</h2>
          <div className="component-example">
            <div className="progress-steps">
              {currentStep && (
                <div className="current-step">
                  <strong>Current Step:</strong> {currentStep}
                </div>
              )}
              
              <div className="step">
                <div className={`step-indicator ${generationProgress.length > 0 ? 'completed' : 'active spinning'}`}>
                  {generationProgress.length > 0 ? '✓' : '○'}
                </div>
                <span className={generationProgress.length > 0 ? 'completed-text' : ''}>Analyzing template structure</span>
              </div>
              <div className="step">
                <div className={`step-indicator ${generationProgress.some(msg => msg.includes('Plan received')) ? 'completed' : planSteps.length > 0 ? 'active' : 'active spinning'}`}>
                  {generationProgress.some(msg => msg.includes('Plan received')) ? '✓' : '○'}
                </div>
                <span className={generationProgress.some(msg => msg.includes('Plan received')) ? 'completed-text' : ''}>Requesting generation plan</span>
              </div>
              
              {planSteps.length > 0 && (
                <>
                  <h3>File Generation Progress:</h3>
                  {planSteps.map((step, index) => (
                    <div key={index} className="step">
                      <div className={`step-indicator ${step.completed ? 'completed' : 'active'} ${step.generating ? 'spinning' : ''}`}>
                        {step.completed ? '✓' : '○'}
                      </div>
                      <span className={step.completed ? 'completed-text' : ''}>{step.file}</span>
                    </div>
                  ))}
                </>
              )}
            </div>
          </div>
        </div>
      </div>
      
      <div className="button-bar">
        <button 
          className="button secondary" 
          onClick={() => {
            vscode.postMessage({ command: 'stopGeneration' });
            setIsGenerating(false);
            setGenerationStopped(true);
          }}
        >
          ⏹️ Stop Generation
        </button>
      </div>
    </div>
  );

  const renderStoppedPage = () => (
    <div id="stopped_page">
      <div className="header-bar">
        <h1>⏹️ Package Generation Stopped</h1>
      </div>
      <div className="component-row">
        <div className="component-container">
          <h2>Generation Cancelled</h2>
          <div className="component-example">
            <p>Package generation was stopped by user request.</p>
            
            {planSteps.length > 0 && (
              <div className="progress-steps">
                <h3>Progress Before Stopping:</h3>
                {planSteps.map((step, index) => (
                  <div key={index} className="step">
                    <div className={`step-indicator ${step.completed ? 'completed' : ''}`}>
                      {step.completed ? '✓' : '○'}
                    </div>
                    <span className={step.completed ? 'completed-text' : ''}>{step.file}</span>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      </div>
      
      <div className="button-bar">
        <button 
          className="button primary" 
          onClick={() => {
            setIsGenerating(false);
            setGenerationProgress([]);
            setGenerationComplete(false);
            setGenerationStopped(false);
            setNaturalLanguageDescription('');
            setTestDescription('');
            setPackageName('');
            setPackageDescription('');
            setPlanSteps([]);
            setCurrentStep('');
          }}
        >
          ✨ Create Another Package
        </button>
        <button 
          className="button secondary" 
          onClick={() => vscode.postMessage({ command: 'cancel' })}
        >
          Close
        </button>
      </div>
    </div>
  );

  const renderCompletePage = () => (
    <div id="complete_page">
      <div className="header-bar">
        <h1>✅ Package Generation Complete!</h1>
      </div>
      <div className="component-row">
        <div className="component-container">
          <h2>🎉 Success</h2>
          <div className="component-example">
            <p>Your ROS 2 package has been successfully generated with AI assistance.</p>
            
            {planSteps.length > 0 && (
              <div className="progress-steps">
                <h3>Generated Files:</h3>
                {planSteps.map((step, index) => (
                  <div key={index} className="step">
                    <div className={`step-indicator ${step.completed ? 'completed' : ''}`}>
                      {step.completed ? '✓' : '○'}
                    </div>
                    <span className={step.completed ? 'completed-text' : ''}>{step.file}</span>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      </div>
      
      <div className="button-bar">
        <button 
          className="button primary" 
          onClick={() => {
            setIsGenerating(false);
            setGenerationProgress([]);
            setGenerationComplete(false);
            setGenerationStopped(false);
            setNaturalLanguageDescription('');
            setTestDescription('');
            setPackageName('');
            setPackageDescription('');
            setPlanSteps([]);
            setCurrentStep('');
          }}
        >
          ✨ Create Another Package
        </button>
        <button 
          className="button secondary" 
          onClick={() => vscode.postMessage({ command: 'cancel' })}
        >
          Close
        </button>
      </div>
    </div>
  );

  return (
    <div className={`app ${isGenerating ? 'generating-page' : ''}`}>
      {!isGenerating && !generationComplete && !generationStopped && renderCreatePackagePage()}
      {isGenerating && renderGeneratingPage()}
      {!isGenerating && generationComplete && renderCompletePage()}
      {!isGenerating && generationStopped && renderStoppedPage()}
    </div>
  );
};

// Initialize the React app
const container = document.getElementById('root');
if (container) {
  const root = createRoot(container);
  root.render(<App />);
}