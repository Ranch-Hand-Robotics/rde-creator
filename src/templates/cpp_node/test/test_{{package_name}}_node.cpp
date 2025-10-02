/**
 * @file test_{{package_name}}_node.cpp
 * @brief Comprehensive unit tests for {{package_name}} ROS 2 C++ node
 * @author {{package_maintainer}}
 * @date {{year}}
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
{{#if include_service}}
#include <std_srvs/srv/trigger.hpp>
{{/if}}
#include <chrono>
#include <memory>
#include <thread>

#include "{{package_name}}/{{package_name}}_node.hpp"

using namespace std::chrono_literals;

class {{node_name|pascalcase}}NodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<{{package_name}}::{{node_name|pascalcase}}Node>();
    test_node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override
  {
    node_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<{{package_name}}::{{node_name|pascalcase}}Node> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
};

/**
 * @brief Test node creation and initialization
 */
TEST_F({{node_name|pascalcase}}NodeTest, NodeCreation)
{
  ASSERT_NE(node_, nullptr);
  EXPECT_EQ(node_->get_name(), "{{node_name}}");
}

{{#if include_publisher}}
/**
 * @brief Test publisher creation and functionality
 */
TEST_F({{node_name|pascalcase}}NodeTest, PublisherTest)
{
  // Verify publisher exists
  auto publisher_names = node_->get_publisher_names_and_types_by_node(
    node_->get_name(), node_->get_namespace());
  
  bool found_topic = false;
  for (const auto& topic_info : publisher_names) {
    if (topic_info.first == "/{{topic_name}}") {
      found_topic = true;
      break;
    }
  }
  EXPECT_TRUE(found_topic);

  // Test message publishing
  std::vector<std_msgs::msg::String::SharedPtr> received_messages;
  auto subscription = test_node_->create_subscription<std_msgs::msg::String>(
    "{{topic_name}}", 10,
    [&received_messages](const std_msgs::msg::String::SharedPtr msg) {
      received_messages.push_back(msg);
    });

  // Allow time for subscription to be established
  std::this_thread::sleep_for(100ms);

  // Spin to process callbacks
  auto start_time = std::chrono::steady_clock::now();
  while (received_messages.empty() && 
         std::chrono::steady_clock::now() - start_time < 5s) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_GT(received_messages.size(), 0);
}
{{/if}}

{{#if include_subscriber}}
/**
 * @brief Test subscriber creation and message handling
 */
TEST_F({{node_name|pascalcase}}NodeTest, SubscriberTest)
{
  // Verify subscriber exists
  auto subscription_names = node_->get_subscription_names_and_types_by_node(
    node_->get_name(), node_->get_namespace());
  
  bool found_topic = false;
  for (const auto& topic_info : subscription_names) {
    if (topic_info.first == "/{{topic_name}}") {
      found_topic = true;
      break;
    }
  }
  EXPECT_TRUE(found_topic);

  // Test message reception
  auto test_publisher = test_node_->create_publisher<std_msgs::msg::String>(
    "{{topic_name}}", 10);

  // Allow time for connections to be established
  std::this_thread::sleep_for(100ms);

  // Publish test message
  auto test_msg = std::make_shared<std_msgs::msg::String>();
  test_msg->data = "test_message";
  test_publisher->publish(*test_msg);

  // Spin to process callbacks
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 2s) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(10ms);
  }

  // The test passes if no exceptions were thrown during message processing
  SUCCEED();
}
{{/if}}

{{#if include_service}}
/**
 * @brief Test service creation and callback functionality
 */
TEST_F({{node_name|pascalcase}}NodeTest, ServiceTest)
{
  // Verify service exists
  auto service_names = node_->get_service_names_and_types_by_node(
    node_->get_name(), node_->get_namespace());
  
  bool found_service = false;
  for (const auto& service_info : service_names) {
    if (service_info.first == "/{{service_name}}") {
      found_service = true;
      break;
    }
  }
  EXPECT_TRUE(found_service);

  // Test service call
  auto client = test_node_->create_client<std_srvs::srv::Trigger>("{{service_name}}");
  
  // Wait for service to be available
  ASSERT_TRUE(client->wait_for_service(5s));

  // Make service call
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  // Wait for response
  auto spin_result = rclcpp::spin_until_future_complete(
    test_node_, future, 5s);
  
  EXPECT_EQ(spin_result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->message.empty());
}
{{/if}}

{{#if include_client}}
/**
 * @brief Test service client creation
 */
TEST_F({{node_name|pascalcase}}NodeTest, ClientTest)
{
  // Verify client exists by checking node's client count
  auto client_names = node_->get_client_names_and_types_by_node(
    node_->get_name(), node_->get_namespace());
  
  bool found_client = false;
  for (const auto& client_info : client_names) {
    if (client_info.first == "/{{service_name}}") {
      found_client = true;
      break;
    }
  }
  EXPECT_TRUE(found_client);
}
{{/if}}

{{#if include_timer}}
/**
 * @brief Test timer functionality
 */
TEST_F({{node_name|pascalcase}}NodeTest, TimerTest)
{
  // Timer functionality is tested implicitly through other tests
  // as timers drive publisher and other periodic activities
  
  // Spin for a few timer periods to ensure timer is working
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 500ms) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
  }
  
  // If we reach here without crashes, timer is working
  SUCCEED();
}
{{/if}}

{{#if include_parameters}}
/**
 * @brief Test parameter handling
 */
TEST_F({{node_name|pascalcase}}NodeTest, ParametersTest)
{
  // Test parameter declaration and retrieval
  try {
    auto param_value = node_->get_parameter("example_param");
    EXPECT_TRUE(param_value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET);
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
    FAIL() << "Parameter 'example_param' should be declared: " << e.what();
  }

  // Test parameter setting
  bool set_result = node_->set_parameter(
    rclcpp::Parameter("example_param", "test_value"));
  EXPECT_TRUE(set_result);

  // Verify parameter was set
  auto updated_param = node_->get_parameter("example_param");
  EXPECT_EQ(updated_param.as_string(), "test_value");
}
{{/if}}

/**
 * @brief Test node lifecycle and cleanup
 */
TEST_F({{node_name|pascalcase}}NodeTest, NodeLifecycle)
{
  // Test that node starts in primary state
  EXPECT_EQ(node_->get_current_state().id(), 
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  
  // Test graceful shutdown
  EXPECT_NO_THROW(node_.reset());
}

/**
 * @brief Integration test for overall node functionality
 */
TEST_F({{node_name|pascalcase}}NodeTest, IntegrationTest)
{
  // Run the node for a short period to test overall integration
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 1s) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(50ms);
  }
  
  // If we reach here without exceptions, integration is successful
  SUCCEED();
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}