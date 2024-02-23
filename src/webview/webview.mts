import { 
  provideVSCodeDesignSystem, 
  vsCodeButton, 
  Button, 
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
}

function handleSecondPageClick() {
  // hide div named "create_package_page", and show div named "create_node_page"
  const createPackagePage = document.getElementById("create_package_page");
  const createNodePage = document.getElementById("create_node_page");
  createPackagePage?.classList.add("hidden");
  createNodePage?.classList.remove("hidden");
}