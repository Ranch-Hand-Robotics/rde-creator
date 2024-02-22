import { 
  provideVSCodeDesignSystem, 
  vsCodeButton, 
  Button, 
  vsCodeRadioGroup, 
  vsCodeRadio, 
  vsCodeTextField,
  vsCodeDropdown,
  vsCodeTextArea } from "@vscode/webview-ui-toolkit";



provideVSCodeDesignSystem().register(vsCodeRadio());
provideVSCodeDesignSystem().register(vsCodeRadioGroup());
provideVSCodeDesignSystem().register(vsCodeButton());
provideVSCodeDesignSystem().register(vsCodeTextField());
provideVSCodeDesignSystem().register(vsCodeDropdown());
provideVSCodeDesignSystem().register(vsCodeTextArea());

const vscode = acquireVsCodeApi();

window.addEventListener("load", main);

function main() {
  // To get improved type annotations/IntelliSense the associated class for
  // a given toolkit component can be imported and used to type cast a reference
  // to the element (i.e. the `as Button` syntax)
  const createNodeButton = document.getElementById("create_node") as Button;
  createNodeButton?.addEventListener("click", handleHowdyClick);
}

function handleHowdyClick() {
  vscode.postMessage({
    command: "hello",
    text: "Hey there partner! 🤠",
  });
}