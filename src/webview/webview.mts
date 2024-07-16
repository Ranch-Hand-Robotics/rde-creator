import { 
  provideVSCodeDesignSystem, 
  vsCodeButton, 
  Button,
  Radio,
  RadioGroup,
  Checkbox,
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

let manifestMap = new Map<string, any>();

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

        // covert the stringified JSON back to a JSON object
        let manifests = JSON.parse(message.manifests);

        // for each manifest in the map, create a new option in the radio group
        for (const [key, value] of Object.entries(manifests) as any) {
          const option = document.createElement("vscode-radio") as Radio;
          option.value = key;
          option.innerText = value.name;
          manifestDropdown.appendChild(option);

          // add the manifest to the map
          manifestMap.set(key, manifests);
        }

        // select the first option by default
        manifestDropdown.value = manifests[0];
        break;
    }
  });
}

function handleSecondPageClick() {
  // hide div named "create_package_page", and show div named "create_node_page"
  const createPackagePage = document.getElementById("create_package_page");
  const createNodePage = document.getElementById("create_node_page");
  const packageType = document.getElementById("ros_package_type") as RadioGroup;

  // Select the manifest based on the selected radio button
  const selectedManifest = manifestMap.get(packageType.value);
  if (selectedManifest === undefined) {
    console.error(`No manifest found for ${packageType.value}`);
    return;
  } else {
    console.log(`Selected manifest: ${selectedManifest}`);


    // populate the second page based on the options in the manifest. a Manifest options map looks like this:
    // options:
    //  - include_urdf: 
    //    name: "Include URDF"
    //    description: "Include a URDF file in the package"
    //    type: "boolean"
    //  - include_launch: 
    //    name: "Include Launch File"
    //    description: "Include a Python Based Launch file in the package"
    //    type: "boolean"
    // a Boolean option would be rendered as a checkbox, a string option would be rendered as a text field, etc.

    const options = selectedManifest.options;

    // Find the section element with the ID IncludeContainer
    const includeContainer = document.getElementById("IncludeContainer");

    // If there are no options, hide the section
    if (options.length === 0) {
      includeContainer?.classList.add("hidden");
    } else {
      includeContainer?.classList.remove("hidden");
    }

    // Clear the children of the IncludeContainer
    includeContainer?.querySelectorAll("*").forEach(child => child.remove());

    // Add a header to the section
    const header = document.createElement("h2");
    header.innerText = "Include:";

    // for each option, add a vscode-checkbox or vscode-text-field to the IncludeContainer
    options.forEach((option: any) => {
      if (option.type === "boolean") {
        const checkbox = document.createElement("vscode-checkbox") as Checkbox;
        checkbox.innerText = option.name;
        includeContainer?.appendChild(checkbox);
      } else {
        const textField = document.createElement("vscode-text-field") as TextField;
        textField.placeholder = option.name;
        includeContainer?.appendChild(textField);
      }
    });
  }


  createPackagePage?.classList.add("hidden");
  createNodePage?.classList.remove("hidden");
}
