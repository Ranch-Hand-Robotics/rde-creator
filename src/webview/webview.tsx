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
  const [currentPage, setCurrentPage] = useState<'create_package' | 'create_node'>('create_package');
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

  useEffect(() => {
    const messageHandler = (event: MessageEvent) => {
      const message = event.data;
      switch (message.command) {
        case 'setManifests':
          vscode.postMessage({ command: 'trace', text: `Received setManifests message: ${JSON.stringify(message)}` });
          const parsedManifests = JSON.parse(message.manifests);
          vscode.postMessage({ command: 'trace', text: `Parsed manifests: ${JSON.stringify(parsedManifests)}` });
          const manifestMap = new Map<string, Manifest>();
          for (const [key, value] of Object.entries(parsedManifests) as [string, any][]) {
            manifestMap.set(key, value);
          }
          vscode.postMessage({ command: 'trace', text: `Manifest map size: ${manifestMap.size}` });
          setManifests(manifestMap);
          if (manifestMap.size > 0) {
            const firstKey = manifestMap.keys().next().value;
            if (firstKey) {
              setSelectedPackageType(firstKey);
            }
          }
          break;
      }
    };

    window.addEventListener('message', messageHandler);
    return () => window.removeEventListener('message', messageHandler);
  }, []);

  const handleNextPage = () => {
    const selectedManifest = manifests.get(selectedPackageType);
    if (!selectedManifest) {
      vscode.postMessage({ command: 'error', text: `No manifest found for ${selectedPackageType}` });
      return;
    }

    vscode.postMessage({ command: 'trace', text: `Selected manifest: ${selectedManifest.name}` });

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

    if (!packageName.trim()) {
      vscode.postMessage({ command: 'error', text: 'Package Name is required' });
      return;
    }

    if (useAI && !naturalLanguageDescription.trim()) {
      vscode.postMessage({ command: 'error', text: 'Natural language description is required for AI generation' });
      return;
    }

    const message: WebviewMessage = {
      command: useAI ? 'createPackageWithAI' : 'createPackage',
      type: selectedManifest.name,
      variables: {
        package_name: packageName,
        package_maintainer: packageMaintainer,
        package_version: packageVersion,
        package_description: packageDescription,
        package_license: packageLicense,
        ...Object.fromEntries(
          Object.entries(includeOptions).map(([key, value]) => [key, value ? 'true' : 'false'])
        )
      },
      ...(useAI && { naturalLanguageDescription })
    };

    vscode.postMessage(message);
  };

  const renderCreatePackagePage = () => (
    <div id="create_package_page">
      <h1>Create new ROS 2 Package:</h1>
      <div className="component-row">
        <div className="component-container">
          <h2>Package type</h2>
          <div className="component-example">
            <fieldset className="radio-group">
              <legend>Package Type:</legend>
              {Array.from(manifests.entries()).map(([key, manifest]) => (
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
              ))}
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
        <button id="second_page_button" onClick={handleNextPage} className="button">
          Next Page
        </button>
      </div>
    </div>
  );

  const renderCreateNodePage = () => {
    const selectedManifest = manifests.get(selectedPackageType);
    const options = selectedManifest?.options || [];

    return (
      <div id="create_node_page">
        <h1>Populate ROS 2 Node:</h1>
        <div className="component-row">
          <div className="component-container" id="IncludeContainer">
            {options.length > 0 && <h2>Include:</h2>}
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
        
        {/* AI Generation Section */}
        <div className="component-row">
          <div className="component-container">
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
                Use AI-powered generation (requires natural language description)
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
                </small>
              </div>
            )}
          </div>
        </div>
        
        <div className="component-row">
          <button id="create_node_button" onClick={handleCreatePackage} className="button">
            {useAI ? 'Generate with AI' : 'Create Package'}
          </button>
        </div>
      </div>
    );
  };

  return (
    <div className={`app ${currentPage === 'create_package' ? '' : 'node-page'}`}>
      {currentPage === 'create_package' ? renderCreatePackagePage() : renderCreateNodePage()}
    </div>
  );
};

// Initialize the React app
const container = document.getElementById('root');
if (container) {
  const root = createRoot(container);
  root.render(<App />);
}