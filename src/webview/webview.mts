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

  const createNodeButton = document.getElementById("create_node_button") as Button;
  createNodeButton?.addEventListener("click", handleCreateNodeClick);

  const secondPageButton = document.getElementById("second_page_button") as Button;
  secondPageButton?.addEventListener("click", handleSecondPageClick);

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
          manifestMap.set(key, value);

          // select the first option by default
          if (manifestDropdown.value === undefined) {
            manifestDropdown.value = key;
          }
        }

        break;
    }
  });
}

function handleCreateNodeClick() {
  const packageType = document.getElementById("ros_package_type") as RadioGroup;
  const selectedManifest = manifestMap.get(packageType.value);
  if (selectedManifest === undefined) {
    vscode.postMessage( {command:"error", text: `No manifest found for ${packageType.value}`});
    vscode.postMessage( {command:"cancel"});
    return;
  } else {
    vscode.postMessage( {command:"trace", text: `Selected manifest: ${selectedManifest.name}`});

    // get the values from the text fields
    const packageName = document.getElementById("package_name") as TextField;
    const packageMaintainer = document.getElementById("package_maintainer") as TextField;
    const packageVersion = document.getElementById("package_version") as TextField;
    const packageDescription = document.getElementById("package_description") as TextArea;
    const packageLicense = document.getElementById("package_license") as TextField;

    if (packageName.textContent == null || packageName.textContent === "") {
      vscode.postMessage( {command:"error", text: "Package Name is required"});
      return;
    }


      // Find the section element with the ID IncludeContainer
      const includeContainer = document.getElementById("IncludeContainer");

      // find all checkboxes in the IncludeContainer
      const checkboxes = includeContainer?.querySelectorAll("vscode-checkbox");

      // iterate over the checkboxes nd add the values to the variables map
      const variables = new Map<string, string>();
      checkboxes?.forEach((el: Element) => {
        const checkbox = el as Checkbox;
        variables.set(checkbox.id, checkbox.checked ? "true" : "false");
      });

      variables.set("package_name", packageName.textContent);
      if (packageMaintainer.textContent !== null) {
        variables.set("package_maintainer", packageMaintainer.textContent);
      }
      if (packageVersion.textContent !== null) {
        variables.set("package_version", packageVersion.textContent);
      }
      if (packageDescription.textContent !== null) {
        variables.set("package_description", packageDescription.textContent);
      }
      if (packageLicense.textContent !== null) {
        variables.set("package_license", packageLicense.textContent);
      }

    
      const reply = {
        command: "createPackage",
        type: selectedManifest.name,
        variables: variables
      };

      vscode.postMessage(reply);


  }
}

function handleSecondPageClick() {
  // hide div named "create_package_page", and show div named "create_node_page"
  const createPackagePage = document.getElementById("create_package_page");
  const createNodePage = document.getElementById("create_node_page");
  const packageType = document.getElementById("ros_package_type") as RadioGroup;

  // Select the manifest based on the selected radio button
  const selectedManifest = manifestMap.get(packageType.value);
  if (selectedManifest === undefined) {
    vscode.postMessage( {command:"error", text: `No manifest found for ${packageType.value}`});
    return;
  } else {
    vscode.postMessage( {command:"trace", text: `Selected manifest: ${selectedManifest.name}`});


    // populate the second page based on the options in the manifest. a Manifest options map looks like this:
    // options:
    //  - varible: include_urdf 
    //    name: "Include URDF"
    //    description: "Include a URDF file in the package"
    //    type: "boolean"
    //  - varible: include_launch 
    //    name: "Include Launch File"
    //    description: "Include a Python Based Launch file in the package"
    //    type: "boolean"
    // a Boolean option would be rendered as a checkbox, a string option would be rendered as a text field, etc.

    const options = selectedManifest.options;

    if (options === undefined) {
    } else {
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
          checkbox.id = option.variable;
          checkbox.innerText = option.name;
          includeContainer?.appendChild(checkbox);
        } else {
          const textField = document.createElement("vscode-text-field") as TextField;
          textField.placeholder = option.name;
          includeContainer?.appendChild(textField);
        }
      });
    }
  }


  createPackagePage?.classList.add("hidden");
  createNodePage?.classList.remove("hidden");
}
