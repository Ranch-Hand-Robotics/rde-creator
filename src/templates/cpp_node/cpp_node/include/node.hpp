/**
 * @file {{node_name}}.hpp
 * @brief Header file for {{package_name}} ROS 2 composable node
 *
 * This file contains the class definition for the {{node_name}} composable node.
 * {{#if include_lifecycle}}
 * This is a lifecycle node with state management capabilities.
 * {{/if}}
 */

#ifndef {{package_name|upper}}_{{node_name|upper}}_HPP_
#define {{package_name|upper}}_{{node_name|upper}}_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
{{#if include_lifecycle}}
#include <rclcpp_lifecycle/lifecycle_node.hpp>
{{/if}}
{{#if include_publisher}}
#include <std_msgs/msg/string.hpp>
{{/if}}
{{#if include_subscriber}}
#include <std_msgs/msg/string.hpp>
{{/if}}
{{#if include_service}}
#include <std_srvs/srv/trigger.hpp>
{{/if}}
{{#if include_client}}
#include <std_srvs/srv/trigger.hpp>
{{/if}}

namespace {{package_name}}
{

{{#if include_lifecycle}}
/**
 * @class {{node_name|pascalcase}}
 * @brief A ROS 2 lifecycle composable node for {{package_name}}
 *
 * This class implements a lifecycle node that can be loaded as a component.
 * It supports state transitions: unconfigured, inactive, active, finalized.
 */
class {{node_name|pascalcase}} : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor for the lifecycle node
   * @param options Node options for component loading
   */
  explicit {{node_name|pascalcase}}(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~{{node_name|pascalcase}}() override = default;

protected:
  /**
   * @brief Configure lifecycle callback
   * @param state Current lifecycle state
   * @return Callback return with success/failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate lifecycle callback
   * @param state Current lifecycle state
   * @return Callback return with success/failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate lifecycle callback
   * @param state Current lifecycle state
   * @return Callback return with success/failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup lifecycle callback
   * @param state Current lifecycle state
   * @return Callback return with success/failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown lifecycle callback
   * @param state Current lifecycle state
   * @return Callback return with success/failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
{{else}}
/**
 * @class {{node_name|pascalcase}}
 * @brief A ROS 2 composable node for {{package_name}}
 *
 * This class implements a standard composable node that can be loaded as a component.
 */
class {{node_name|pascalcase}} : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the composable node
   * @param options Node options for component loading
   */
  explicit {{node_name|pascalcase}}(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~{{node_name|pascalcase}}() override = default;

private:
{{/if}}

{{#if include_publisher}}
  /**
   * @brief Publisher for string messages
   */
  rclcpp{{#if include_lifecycle}}_lifecycle{{/if}}::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
{{/if}}

{{#if include_subscriber}}
  /**
   * @brief Subscriber for string messages
   */
  rclcpp{{#if include_lifecycle}}_lifecycle{{/if}}::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
{{/if}}

{{#if include_service}}
  /**
   * @brief Service server for trigger requests
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
{{/if}}

{{#if include_client}}
  /**
   * @brief Service client for trigger requests
   */
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
{{/if}}

{{#if include_timer}}
  /**
   * @brief Timer for periodic tasks
   */
  rclcpp::TimerBase::SharedPtr timer_;
{{/if}}

{{#if include_subscriber}}
  /**
   * @brief Callback function for subscriber
   * @param msg Received message
   */
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);
{{/if}}

{{#if include_service}}
  /**
   * @brief Callback function for service requests
   * @param request Service request
   * @param response Service response
   */
  void service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
{{/if}}

{{#if include_client}}
  /**
   * @brief Call the service
   */
  void call_service();
{{/if}}

{{#if include_timer}}
  /**
   * @brief Timer callback function
   */
  void timer_callback();
{{/if}}

{{#if include_client}}
  /**
   * @brief Service response callback
   * @param future Future containing the service response
   */
  void service_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
{{/if}}
};

}  // namespace {{package_name}}

#endif  // {{package_name|upper}}_{{node_name|upper}}_HPP_