/**
 * @file test/setup.js
 * @brief Jest test setup for ROS 2 Node.js tests
 */

// Increase timeout for ROS 2 operations
jest.setTimeout(15000);

// Mock console methods to reduce test noise if needed
// global.console = {
//   ...console,
//   // Uncomment to suppress logs during testing
//   // log: jest.fn(),
//   // debug: jest.fn(),
//   // info: jest.fn(),
//   // warn: jest.fn(),
//   // error: jest.fn(),
// };

// Global test utilities
global.delay = (ms) => new Promise(resolve => setTimeout(resolve, ms));

// Setup and teardown helpers
beforeAll(async () => {
  // Global setup if needed
});

afterAll(async () => {
  // Global cleanup if needed
});