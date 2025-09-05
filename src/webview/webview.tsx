import React, { useState, useEffect } from 'react';
import { createRoot } from 'react-dom/client';

// VS Code API
declare const acquireVsCodeApi: () => any;
const vscode = acquireVsCodeApi();

interface Manifest {
  name: string;
  options?: Array<{
    variable: string;
    name: string;
    description: string;
    type: string;
  }>;
}

interface WebviewMessage {
  command: string;
  text?: string;
  type?: string;
  variables?: Record<string, string>;
  naturalLanguageDescription?: string;
}

const App: React.FC = () => {
  const [currentPage, setCurrentPage] = useState<'create_package' | 'create_node' | 'generating'>('create_package');
  const [manifests, setManifests] = useState<Map<string, Manifest>>(new Map());
  const [selectedPackageType, setSelectedPackageType] = useState<string>('');
  const [packageName, setPackageName] = useState('');
  const [packageMaintainer, setPackageMaintainer] = useState('');
  const [packageVersion, setPackageVersion] = useState('');
  const [packageDescription, setPackageDescription] = useState('');
  const [packageLicense, setPackageLicense] = useState('');
  const [includeOptions, setIncludeOptions] = useState<Record<string, boolean>>({});
  const [naturalLanguageDescription, setNaturalLanguageDescription] = useState('');
  const [useAI, setUseAI] = useState(false);
  const [isLoadingManifests, setIsLoadingManifests] = useState(true);
  const [generationProgress, setGenerationProgress] = useState<string[]>([]);
  const [isGenerating, setIsGenerating] = useState(false);
  const [manifestLoadStartTime, setManifestLoadStartTime] = useState<number | null>(null);

  useEffect(() => {
    // Notify the extension that the webview is loaded and ready
    vscode.postMessage({ command: 'webviewLoaded' });
    
    setManifestLoadStartTime(Date.now());
    
    const messageHandler = (event: MessageEvent) => {
      const message = event.data;
      switch (message.command) {
        case 'setManifests':
          const parsedManifests = JSON.parse(message.manifests);
          const manifestMap = new Map<string, Manifest>();
          for (const [key, value] of Object.entries(parsedManifests) as [string, any][]) {
            manifestMap.set(key, value);
          }
          setManifests(manifestMap);
          setIsLoadingManifests(false);
          setManifestLoadStartTime(null);
          if (manifestMap.size > 0) {
            const firstKey = manifestMap.keys().next().value;
            if (firstKey) {
              setSelectedPackageType(firstKey);
            }
          }
          break;
        case 'error':
          setIsLoadingManifests(false);
          setManifestLoadStartTime(null);
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

  // Add timeout for manifest loading
  useEffect(() => {
    if (manifestLoadStartTime && isLoadingManifests) {
      const timeoutId = setTimeout(() => {
        const elapsed = Date.now() - manifestLoadStartTime;
        if (elapsed > 10000) { // 10 seconds timeout
          setIsLoadingManifests(false);
          setManifestLoadStartTime(null);
          vscode.postMessage({ command: 'error', text: 'Manifest loading timed out. Please try reopening the panel.' });
        }
      }, 10000);

      return () => clearTimeout(timeoutId);
    }
  }, [manifestLoadStartTime, isLoadingManifests]);

  const handleNextPage = () => {
    const selectedManifest = manifests.get(selectedPackageType);
    if (!selectedManifest) {
      vscode.postMessage({ command: 'error', text: `No manifest found for ${selectedPackageType}` });
      return;
    }

    const options: Record<string, boolean> = {};
    if (selectedManifest.options) {
      selectedManifest.options.forEach(option => {
        if (option.type === 'boolean') {
          options[option.variable] = false;
        }
      });
    }
    setIncludeOptions(options);
    setCurrentPage('create_node');
  };

  const handleCreatePackage = () => {
    const selectedManifest = manifests.get(selectedPackageType);
    if (!selectedManifest) {
      vscode.postMessage({ command: 'error', text: `No manifest found for ${selectedPackageType}` });
      vscode.postMessage({ command: 'cancel' });
      return;
    }

    // Use placeholder values if fields are empty
    const finalPackageName = packageName.trim() || "my_package";
    const finalPackageMaintainer = packageMaintainer.trim() || "robots@example.com";
    const finalPackageVersion = packageVersion.trim() || "0.0.0";
    const finalPackageDescription = packageDescription.trim() || "This is a sample description";
    const finalPackageLicense = packageLicense.trim() || "MIT";

    // Use package description as natural language description if it's been modified
    let finalNaturalLanguageDescription = naturalLanguageDescription.trim();
    
    // If package description has been modified (not default), use it as natural language description
    if (finalPackageDescription && finalPackageDescription !== "This is a sample description") {
      if (!finalNaturalLanguageDescription) {
        // Use package description if natural language is empty
        finalNaturalLanguageDescription = finalPackageDescription;
      } else {
        // Combine both if both are provided
        finalNaturalLanguageDescription = `${finalPackageDescription}\n\n${finalNaturalLanguageDescription}`;
      }
    } else if (!finalNaturalLanguageDescription) {
      // Use default placeholder if both are empty
      finalNaturalLanguageDescription = "Describe what you want your ROS 2 node to do. For example: 'Create a publisher node that publishes sensor data at 10Hz and subscribes to control commands'";
    }

    if (!finalPackageName) {
      vscode.postMessage({ command: 'error', text: 'Package Name is required' });
      return;
    }

    if (useAI && !finalNaturalLanguageDescription.trim()) {
      vscode.postMessage({ command: 'error', text: 'Natural language description is required for AI generation' });
      return;
    }

    // For AI generation, we need either natural language description or package description
    if (useAI && !finalNaturalLanguageDescription.trim() && finalPackageDescription === "This is a sample description") {
      vscode.postMessage({ command: 'error', text: 'Either natural language description or a custom package description is required for AI generation' });
      return;
    }

    // Combine package description with natural language description for AI
    let combinedDescription = finalNaturalLanguageDescription;
    if (finalPackageDescription && finalPackageDescription !== "This is a sample description") {
      combinedDescription = `Package Description: ${finalPackageDescription}\n\nDetailed Requirements: ${finalNaturalLanguageDescription}`;
    }

    const message: WebviewMessage = {
      command: useAI ? 'createPackageWithAI' : 'createPackage',
      type: selectedManifest.name,
      variables: {
        package_name: finalPackageName,
        package_maintainer: finalPackageMaintainer,
        package_version: finalPackageVersion,
        package_description: finalPackageDescription,
        package_license: finalPackageLicense,
        ...Object.fromEntries(
          Object.entries(includeOptions).map(([key, value]) => [key, value ? 'true' : 'false'])
        )
      },
      ...(useAI && { naturalLanguageDescription: finalNaturalLanguageDescription })
    };

    if (useAI) {
      setIsGenerating(true);
      setGenerationProgress([]);
      setCurrentPage('generating');
    }

    vscode.postMessage(message);
  };

  const handleBack = () => {
    if (currentPage === 'create_node') {
      setCurrentPage('create_package');
    } else if (currentPage === 'generating') {
      setCurrentPage('create_node');
    } else {
      // On the first page, close the panel
      vscode.postMessage({ command: 'cancel' });
    }
  };

  // Hook into browser navigation
  useEffect(() => {
    const handlePopState = (event: PopStateEvent) => {
      event.preventDefault();
      handleBack();
    };

    // Set up initial state for navigation
    if (typeof window !== 'undefined' && window.history) {
      window.history.replaceState({ page: currentPage }, '', '');
    }

    window.addEventListener('popstate', handlePopState);
    return () => window.removeEventListener('popstate', handlePopState);
  }, [currentPage]);

  // Update browser history when page changes
  useEffect(() => {
    if (typeof window !== 'undefined' && window.history) {
      window.history.pushState({ page: currentPage }, '', '');
    }
  }, [currentPage]);

  const renderCreatePackagePage = () => (
    <div id="create_package_page">
      <div className="header-bar">
        <button 
          className="back-button" 
          onClick={handleBack}
          title="Close"
        >
          ← Close
        </button>
        <h1>Create new ROS 2 Package:</h1>
      </div>
      <div className="component-row">
        <div className="component-container">
          <h2>Package type</h2>
          <div className="component-example">
            <fieldset className="radio-group">
              <legend>Package Type:</legend>
              {isLoadingManifests ? (
                <p>Loading package templates...</p>
              ) : manifests.size === 0 ? (
                <div>
                  <p>No package templates found. Please check the extension installation.</p>
                  <button 
                    className="button" 
                    onClick={() => {
                      setIsLoadingManifests(true);
                      setManifestLoadStartTime(Date.now());
                      vscode.postMessage({ command: 'retryManifests' });
                    }}
                    style={{ marginTop: '1rem' }}
                  >
                    Retry Loading
                  </button>
                </div>
              ) : (
                Array.from(manifests.entries()).map(([key, manifest]) => (
                  <label key={key} className="radio-option">
                    <input
                      type="radio"
                      name="packageType"
                      value={key}
                      checked={selectedPackageType === key}
                      onChange={(e) => setSelectedPackageType(e.target.value)}
                    />
                    {manifest.name}
                  </label>
                ))
              )}
            </fieldset>
          </div>
        </div>
        <div className="component-container">
          <h2>Package Metadata</h2>
          <div className="component-example">
            <div className="form-field">
              <label htmlFor="package_name">Name</label>
              <input
                id="package_name"
                type="text"
                placeholder="my_package"
                value={packageName}
                onChange={(e) => setPackageName(e.target.value)}
                className="text-field"
              />
            </div>
            <div className="form-field">
              <label htmlFor="package_maintainer">Maintainer</label>
              <input
                id="package_maintainer"
                type="text"
                placeholder="robots@example.com"
                value={packageMaintainer}
                onChange={(e) => setPackageMaintainer(e.target.value)}
                className="text-field"
              />
            </div>
            <div className="form-field">
              <label htmlFor="package_version">Version</label>
              <input
                id="package_version"
                type="text"
                placeholder="0.0.0"
                value={packageVersion}
                onChange={(e) => setPackageVersion(e.target.value)}
                className="text-field"
              />
            </div>
            <div className="form-field">
              <label htmlFor="package_description">Description</label>
              <textarea
                id="package_description"
                placeholder="This is a sample description"
                rows={5}
                cols={50}
                value={packageDescription}
                onChange={(e) => setPackageDescription(e.target.value)}
                className="text-area"
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
        </div>
      </div>
      <div className="component-row">
        <button id="second_page_button" onClick={handleNextPage} className="button" disabled={isLoadingManifests || manifests.size === 0}>
          {isLoadingManifests ? 'Loading...' : 'Next Page'}
        </button>
      </div>
    </div>
  );

  const renderCreateNodePage = () => {
    const selectedManifest = manifests.get(selectedPackageType);
    const options = selectedManifest?.options || [];

    return (
      <div id="create_node_page">
        <div className="header-bar">
          <button 
            className="back-button" 
            onClick={handleBack}
            title="Back to Package Selection"
          >
            ← Back
          </button>
          <h1>Populate ROS 2 Node:</h1>
        </div>
        {options.length > 0 && (
          <div className="component-container full-width" id="IncludeContainer">
            <h2>Include:</h2>
            <div className="component-example">
              {options.map((option) => {
              if (option.type === 'boolean') {
                return (
                  <div key={option.variable} className="form-field">
                    <label className="checkbox-label">
                      <input
                        type="checkbox"
                        id={option.variable}
                        checked={includeOptions[option.variable] || false}
                        onChange={(e) => setIncludeOptions(prev => ({
                          ...prev,
                          [option.variable]: e.target.checked
                        }))}
                        className="checkbox"
                      />
                      {option.name}
                    </label>
                  </div>
                );
              } else {
                return (
                  <div key={option.variable} className="form-field">
                    <label htmlFor={option.variable}>{option.name}</label>
                    <input
                      id={option.variable}
                      type="text"
                      placeholder={option.name}
                      className="text-field"
                    />
                  </div>
                );
              }
            })}
            </div>
          </div>
        )}
        
        {/* AI Generation Section */}
        <div className="component-container full-width">
          <h2>Generation Mode:</h2>
          <div className="form-field">
            <label className="checkbox-label">
              <input
                type="checkbox"
                id="useAI"
                checked={useAI}
                onChange={(e) => setUseAI(e.target.checked)}
                className="checkbox"
              />
              Use AI-powered generation (provide description in either the Package Description field above or the detailed description below)
            </label>
          </div>
          
          {useAI && (
            <div className="form-field">
              <label htmlFor="naturalLanguageDescription">Natural Language Description</label>
              <textarea
                id="naturalLanguageDescription"
                placeholder="Describe what you want your ROS 2 node to do. For example: 'Create a publisher node that publishes sensor data at 10Hz and subscribes to control commands'"
                rows={6}
                cols={50}
                value={naturalLanguageDescription}
                onChange={(e) => setNaturalLanguageDescription(e.target.value)}
                className="text-area"
              />
              <small style={{ color: '#666', fontSize: '0.9em' }}>
                Provide a detailed description of the ROS 2 node's functionality, topics, services, and behavior. 
                You can also provide details in the Package Description field above.
              </small>
            </div>
          )}
        </div>
        
        <div className="component-row">
          <button id="create_node_button" onClick={handleCreatePackage} className="button">
            {useAI ? 'Generate with AI' : 'Create Package'}
          </button>
        </div>
      </div>
    );
  };

  const renderGeneratingPage = () => (
    <div id="generating_page">
      <div className="header-bar">
        <button 
          className="back-button" 
          onClick={handleBack}
          title="Back to Node Configuration"
          disabled={isGenerating}
        >
          ← Back
        </button>
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
    <div className={`app ${currentPage === 'create_package' ? '' : currentPage === 'create_node' ? 'node-page' : 'generating-page'}`}>
      {currentPage === 'create_package' ? renderCreatePackagePage() : 
       currentPage === 'create_node' ? renderCreateNodePage() : 
       renderGeneratingPage()}
    </div>
  );
};

// Initialize the React app
const container = document.getElementById('root');
if (container) {
  const root = createRoot(container);
  root.render(<App />);
}