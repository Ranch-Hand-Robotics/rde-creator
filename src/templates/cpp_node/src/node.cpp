/**
 * @file {{node_name}}.cpp
 * @brief Implementation of {{package_name}} ROS 2 composable node
 *
 * This file contains the implementation of the {{node_name}} composable node.
 * {{#if include_lifecycle}}
 * This is a lifecycle node with state management capabilities.
 * {{/if}}
 */

#include "{{package_name}}/{{node_name}}.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

using namespace std::chrono_literals;

namespace {{package_name}}
{

{{#if include_lifecycle}}
{{node_name|pascalcase}}::{{node_name|pascalcase}}(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("{{node_name}}", options)
{
  RCLCPP_INFO(this->get_logger(), "{{node_name}} lifecycle node initialized");

  {{#if include_parameters}}
  // Declare parameters
  this->declare_parameter("example_param", "default_value");
  RCLCPP_INFO(this->get_logger(), "Parameters declared");
  {{/if}}
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
{{node_name|pascalcase}}::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring {{node_name}}");

  {{#if include_publisher}}
  // Create lifecycle publisher
  publisher_ = this->create_publisher<std_msgs::msg::String>("{{topic_name}}", 10);
  RCLCPP_INFO(this->get_logger(), "Publisher created for topic: {{topic_name}}");
  {{/if}}

  {{#if include_subscriber}}
  // Create lifecycle subscription
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "{{topic_name}}", 10,
    std::bind(&{{node_name|pascalcase}}::topic_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscriber created for topic: {{topic_name}}");
  {{/if}}

  {{#if include_service}}
  // Create service server
  service_ = this->create_service<std_srvs::srv::Trigger>(
    "{{service_name}}",
    std::bind(&{{node_name|pascalcase}}::service_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service server created for: {{service_name}}");
  {{/if}}

  {{#if include_client}}
  // Create service client
  client_ = this->create_client<std_srvs::srv::Trigger>("{{service_name}}");
  RCLCPP_INFO(this->get_logger(), "Service client created for: {{service_name}}");
  {{/if}}

  {{#if include_timer}}
  // Create timer
  timer_ = this->create_wall_timer(
    {{timer_period}}s, std::bind(&{{node_name|pascalcase}}::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Timer created with period: {{timer_period}} seconds");
  {{/if}}

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
{{node_name|pascalcase}}::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating {{node_name}}");

  {{#if include_publisher}}
  publisher_->on_activate();
  {{/if}}

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
{{node_name|pascalcase}}::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating {{node_name}}");

  {{#if include_publisher}}
  publisher_->on_deactivate();
  {{/if}}

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
{{node_name|pascalcase}}::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up {{node_name}}");

  {{#if include_publisher}}
  publisher_.reset();
  {{/if}}
  {{#if include_subscriber}}
  subscription_.reset();
  {{/if}}
  {{#if include_service}}
  service_.reset();
  {{/if}}
  {{#if include_client}}
  client_.reset();
  {{/if}}
  {{#if include_timer}}
  timer_.reset();
  {{/if}}

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
{{node_name|pascalcase}}::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down {{node_name}}");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

{{else}}
{{node_name|pascalcase}}::{{node_name|pascalcase}}(const rclcpp::NodeOptions & options)
: rclcpp::Node("{{node_name}}", options)
{
  RCLCPP_INFO(this->get_logger(), "{{node_name}} composable node initialized");

  {{#if include_parameters}}
  // Declare parameters
  this->declare_parameter("example_param", "default_value");
  RCLCPP_INFO(this->get_logger(), "Parameters declared");
  {{/if}}

  {{#if include_publisher}}
  // Create publisher
  publisher_ = this->create_publisher<std_msgs::msg::String>("{{topic_name}}", 10);
  RCLCPP_INFO(this->get_logger(), "Publisher created for topic: {{topic_name}}");
  {{/if}}

  {{#if include_subscriber}}
  // Create subscription
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "{{topic_name}}", 10,
    std::bind(&{{node_name|pascalcase}}::topic_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscriber created for topic: {{topic_name}}");
  {{/if}}

  {{#if include_service}}
  // Create service server
  service_ = this->create_service<std_srvs::srv::Trigger>(
    "{{service_name}}",
    std::bind(&{{node_name|pascalcase}}::service_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service server created for: {{service_name}}");
  {{/if}}

  {{#if include_client}}
  // Create service client
  client_ = this->create_client<std_srvs::srv::Trigger>("{{service_name}}");
  RCLCPP_INFO(this->get_logger(), "Service client created for: {{service_name}}");
  {{/if}}

  {{#if include_timer}}
  // Create timer
  timer_ = this->create_wall_timer(
    {{timer_period}}s, std::bind(&{{node_name|pascalcase}}::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Timer created with period: {{timer_period}} seconds");
  {{/if}}
}
{{/if}}

{{#if include_subscriber}}
void {{node_name|pascalcase}}::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());

  // Process the received message
  // Add your message processing logic here
  std::string processed_data = "Processed: " + msg->data;

  {{#if include_publisher}}
  // Optionally publish processed data
  auto response_msg = std::make_unique<std_msgs::msg::String>();
  response_msg->data = processed_data;
  {{#if include_lifecycle}}
  if (publisher_->is_activated()) {
    publisher_->publish(std::move(response_msg));
  }
  {{else}}
  publisher_->publish(std::move(response_msg));
  {{/if}}
  RCLCPP_DEBUG(this->get_logger(), "Published processed message: '%s'", processed_data.c_str());
  {{/if}}
}
{{/if}}

{{#if include_service}}
void {{node_name|pascalcase}}::service_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Suppress unused parameter warning

  RCLCPP_INFO(this->get_logger(), "Service request received");

  // Process the service request
  // Add your service logic here
  response->success = true;
  response->message = "Service executed successfully";

  RCLCPP_INFO(this->get_logger(), "Service response: '%s'", response->message.c_str());
}
{{/if}}

{{#if include_client}}
void {{node_name|pascalcase}}::call_service()
{
  if (!client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->get_logger(), "Service not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client_->async_send_request(request);
  future.wait();  // Wait for response

  if (future.valid()) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Service response: '%s'", response->message.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
  }
}

void {{node_name|pascalcase}}::service_response_callback(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  try {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Service response: '%s'", response->message.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
  }
}
{{/if}}

{{#if include_timer}}
void {{node_name|pascalcase}}::timer_callback()
{
  RCLCPP_DEBUG(this->get_logger(), "Timer callback executed");

  {{#if include_publisher}}
  // Publish a message on timer
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Hello from {{node_name}} at " + std::to_string(this->now().seconds());
  {{#if include_lifecycle}}
  if (publisher_->is_activated()) {
    publisher_->publish(std::move(msg));
  }
  {{else}}
  publisher_->publish(std::move(msg));
  {{/if}}
  RCLCPP_DEBUG(this->get_logger(), "Published timer message");
  {{/if}}

  {{#if include_parameters}}
  // Check parameter changes
  std::string param_value = this->get_parameter("example_param").as_string();
  RCLCPP_DEBUG(this->get_logger(), "Parameter value: '%s'", param_value.c_str());
  {{/if}}

  {{#if include_client}}
  // Optionally call service on timer
  static int counter = 0;
  if (++counter % 10 == 0) {  // Every 10 timer callbacks
    call_service();
  }
  {{/if}}
}
{{/if}}

}  // namespace {{package_name}}

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE({{package_name}}::{{node_name|pascalcase}})