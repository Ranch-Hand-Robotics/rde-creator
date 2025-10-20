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

  const [availableModels, setAvailableModels] = useState<LanguageModel[]>([]);
  const [selectedModel, setSelectedModel] = useState<string>('');

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
          } catch (error) {
            console.error('Error parsing models:', error);
          }
          break;
        case 'aiProgress':
          setGenerationProgress(prev => [...prev, message.text]);
          break;
        case 'aiComplete':
          setIsGenerating(false);
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
      testDescription: testDescription.trim(),
      selectedModel: selectedModel
    };

    setIsGenerating(true);
    setGenerationProgress([]);
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
            <label htmlFor="testDescription">🧪 Test Requirements (Optional; leave blank for no tests)</label>
            <textarea
              id="testDescription"
              placeholder="Example: Create unit tests for the IMU data processing, verify calibration service functionality, test error handling for disconnected sensors, and validate parameter updates."
              rows={3}
              value={testDescription}
              onChange={(e) => setTestDescription(e.target.value)}
              className="text-area"
            />
          </div>
        </div>

        {/* AI Model Selection */}
        <div className="component-container">
          <h2>🤖 AI Model Selection</h2>
          <div className="form-field">
            <label htmlFor="modelSelect">Choose AI Model</label>
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
              💡 Select the AI model to use for generating your ROS 2 package. Different models may produce different results.
            </small>
          </div>
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
        <h1>Generating ROS 2 Package with AI</h1>
      </div>
      <div className="component-row">
        <div className="component-container">
          <h2>AI Generation Progress</h2>
          <div className="component-example">
            <div className="progress-container">
              <div className="progress-spinner"></div>
              <p>AI is analyzing your requirements and generating the ROS 2 package...</p>
            </div>
            
            {generationProgress.length > 0 && (
              <div className="progress-log">
                <h3>Progress Log:</h3>
                <div className="log-entries">
                  {generationProgress.map((message, index) => (
                    <div key={index} className="log-entry">
                      <span className="log-timestamp">{new Date().toLocaleTimeString()}</span>
                      <span className="log-message">{message}</span>
                    </div>
                  ))}
                </div>
              </div>
            )}
            
            <div className="progress-steps">
              <div className="step">
                <div className="step-indicator active"></div>
                <span>Analyzing template structure</span>
              </div>
              <div className="step">
                <div className={`step-indicator ${generationProgress.some(msg => msg.includes('prompt') || msg.includes('language')) ? 'active' : ''}`}></div>
                <span>Processing natural language description</span>
              </div>
              <div className="step">
                <div className={`step-indicator ${generationProgress.some(msg => msg.includes('model') || msg.includes('response')) ? 'active' : ''}`}></div>
                <span>Generating ROS 2 code and configuration</span>
              </div>
              <div className="step">
                <div className={`step-indicator ${generationProgress.some(msg => msg.includes('files') || msg.includes('created')) ? 'active' : ''}`}></div>
                <span>Creating package files</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );

  return (
    <div className={`app ${isGenerating ? 'generating-page' : ''}`}>
      {renderCreatePackagePage()}
      {isGenerating && renderGeneratingPage()}
    </div>
  );
};

// Initialize the React app
const container = document.getElementById('root');
if (container) {
  const root = createRoot(container);
  root.render(<App />);
}