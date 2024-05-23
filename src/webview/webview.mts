import { 
  provideVSCodeDesignSystem, 
  vsCodeButton, 
  Button,
  Radio,
  RadioGroup,
  TextArea,
  TextField, 
  Dropdown,
  vsCodeRadioGroup, 
  vsCodeRadio, 
  vsCodeTextField,
  vsCodeDropdown,
  vsCodeTextArea,
  vsCodeCheckbox } from "@vscode/webview-ui-toolkit";



provideVSCodeDesignSystem().register(vsCodeRadio());
provideVSCodeDesignSystem().register(vsCodeRadioGroup());
provideVSCodeDesignSystem().register(vsCodeButton());
provideVSCodeDesignSystem().register(vsCodeTextField());
provideVSCodeDesignSystem().register(vsCodeDropdown());
provideVSCodeDesignSystem().register(vsCodeTextArea());
provideVSCodeDesignSystem().register(vsCodeCheckbox());

const vscode = acquireVsCodeApi();

window.addEventListener("load", main);

function main() {
  // To get improved type annotations/IntelliSense the associated class for
  // a given toolkit component can be imported and used to type cast a reference
  // to the element (i.e. the `as Button` syntax)
  const createNodeButton = document.getElementById("second_page_button") as Button;
  createNodeButton?.addEventListener("click", handleSecondPageClick);

  window.addEventListener('message', event => {
    const message = event.data; // The JSON data our extension sent
    switch (message.command) {
      case "setManifests":
        const manifestDropdown = document.getElementById("ros_package_type") as RadioGroup;
        // for each manifest in the map, create a new option in the radio group
        for (const [key, value] of Object.entries(message.manifests) {
          const option = document.createElement("vscode-radio") as Radio;
          option.value = key;
          option.innerText = key;
          manifestDropdown.appendChild(option);
        }



    }
  });
}

function handleSecondPageClick() {
  // hide div named "create_package_page", and show div named "create_node_page"
  const createPackagePage = document.getElementById("create_package_page");
  const createNodePage = document.getElementById("create_node_page");
  const packageType = document.getElementById("ros_package_type") as RadioGroup;

  if (packageType.value === "type_resource") {
    const packageName = document.getElementById("package_name") as TextField;
    const packageMaintainer = document.getElementById("package_maintainer") as TextField;
    const packageVersion = document.getElementById("package_version") as TextField;
    const packageDescription = document.getElementById("package_Description") as TextArea;
    const packageLicense = document.getElementById("package_license") as TextField;

    vscode.postMessage({ 
      command: "createResourcePackage",  
      packageName: packageName.value, 
      packageMaintainer: packageMaintainer.value, 
      packageVersion: packageVersion.value, 
      packageDescription: packageDescription.value,
      packageLicense: packageLicense.value
    });
  } else {
    createPackagePage?.classList.add("hidden");
    createNodePage?.classList.remove("hidden");
  }
}
