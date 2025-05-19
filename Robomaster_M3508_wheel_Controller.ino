/**
 * RoboMaster M3508 Motor Control Program (micro-ROS with Velocity Control and Logging)
 * - Controls 4 RoboMaster M3508 DC motors via C620 controllers.
 * - Teensy 4.0 with a single MCP2515 for SPI-based CAN communication.
 * - Operates in Velocity control mode only.
 * - Subscribes to ROS2 cmd_vel topic via micro-ROS.
 * - Publishes motor 1's speed (RPM) on "motor_rotate_speed" topic.
 * - Implements micro-ROS agent connection checking and re-establishment.
 * - Uses RCUTILS logging macros for output.
 *
 * Version: 0.6
 * Changes from 0.5:
 * - Introduced named constants for magic numbers.
 * - Refined PID delta time usage.
 * - Renamed twist_msg_subscriber to twist_msg_buffer for clarity.
 * - Enabled and refined publisher error logging.
 * - Minor improvements to CAN initialization.
 */

#include <SPI.h>
#include <mcp_can.h>

// micro-ROS includes
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>      // For publishing motor speed
#include <rmw_microros/rmw_microros.h> // For rmw_uros_ping_agent
#include <rcutils/logging_macros.h>  // For RCUTILS_LOG_INFO_NAMED
#include <rcutils/time.h>            // Often needed by logging utilities

// C620 Controller SS Pin Configuration (Single CAN Interface)
#define SPI_CS_PIN1 10  // SS Pin for the MCP2515 CAN Controller

// MCP_CAN instance for the single CAN interface
MCP_CAN CAN1(SPI_CS_PIN1);

// CAN Bus Data Variables
#define CAN_ID_TX 0x200      // ID for sending motor commands (to all motors on the bus)
#define CAN_ID_RX_BASE 0x200 // Base ID for motor feedback (e.g., 0x201 for motor 1, 0x202 for motor 2, etc.)
#define MCP_CAN_KBPS CAN_1000KBPS
#define MCP_CAN_SPI_MHZ MCP_8MHZ
#define CAN_INIT_RETRY_DELAY_MS 100

// Number of Motors
#define MOTOR_COUNT 4

// Motor Parameters
#define MAX_CURRENT 16384  // Maximum current value for C620 (±16384 for M3508)
#define MAX_VELOCITY 8000  // Maximum motor speed (RPM) for M3508 (approx. 8000-9000 RPM)
#define GEAR_RATIO 19.2f   // Gear ratio for M3508 (3591/187 = 19.20)

// PID Gains (Require tuning for Velocity Control)
float velocity_kp[MOTOR_COUNT] = {10.0, 10.0, 10.0, 10.0};
float velocity_ki[MOTOR_COUNT] = {0.1, 0.1, 0.1, 0.1};
float velocity_kd[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};

// PID Calculation Variables (Velocity Control)
float velocity_error[MOTOR_COUNT] = {0};
float velocity_error_prev[MOTOR_COUNT] = {0};
float velocity_error_sum[MOTOR_COUNT] = {0};
#define PID_INTEGRAL_LIMIT 1000.0f

// Motor Target Values (Velocity Control)
float target_velocity[MOTOR_COUNT] = {0}; // Target motor RPM

// Motor Current Values (Feedback)
int16_t current_angle[MOTOR_COUNT] = {0};      // Raw encoder angle (0-8191)
int16_t current_velocity[MOTOR_COUNT] = {0};   // Current motor RPM
int16_t current_torque[MOTOR_COUNT] = {0};     // Current motor torque (raw value)
uint8_t current_temperature[MOTOR_COUNT] = {0}; // Current motor temperature (Celsius)

// Variables for Continuous Angle Tracking
int16_t last_angle[MOTOR_COUNT] = {0};         // Previous raw encoder angle
int32_t multi_angle[MOTOR_COUNT] = {0};        // Continuous multi-turn angle (encoder ticks)
int32_t angle_difference[MOTOR_COUNT] = {0};
#define ENCODER_MAX_VALUE 8191
#define ENCODER_HALF_VALUE 4096 // (ENCODER_MAX_VALUE / 2) + 1, for wrap-around detection
#define ENCODER_FULL_RANGE 8192 // ENCODER_MAX_VALUE + 1

// cmd_vel input (updated by micro-ROS)
float cmd_vel_linear_x = 0.0;
float cmd_vel_linear_y = 0.0;
float cmd_vel_angular_z = 0.0;

// Timer Variables
unsigned long last_cmd_time = 0;
unsigned long last_control_time = 0;
#define CONTROL_LOOP_PERIOD_MS 10UL
const float CONTROL_LOOP_PERIOD_S = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;
#define CMD_VEL_TIMEOUT_MS 500UL

// Mecanum Wheel Parameters (Adjust based on robot dimensions)
#define WHEEL_RADIUS 0.076f  // Wheel radius in meters (e.g., 76mm)
#define ROBOT_WIDTH 0.3f     // Robot width in meters (distance between left and right wheels)
#define ROBOT_LENGTH 0.3f    // Robot length in meters (distance between front and rear wheels)

// micro-ROS Connection State
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// micro-ROS Agent Ping Parameters
#define AGENT_PING_INTERVAL_MS_WAITING 500UL
#define AGENT_PING_INTERVAL_MS_CONNECTED 200UL
#define AGENT_PING_TIMEOUT_MS 100
#define AGENT_PING_ATTEMPTS 1

// micro-ROS Variables
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber_cmd_vel;
geometry_msgs__msg__Twist twist_msg_buffer; // Buffer for incoming twist messages

rcl_publisher_t motor_speed_publisher;
std_msgs__msg__Int32 motor_speed_msg; // Message for publishing motor speed

#define EXECUTOR_SPIN_TIMEOUT_NS RCL_MS_TO_NS(5) // Timeout for rclc_executor_spin_some

// LED for status indication (usually built-in LED)
#define LED_PIN 13

// ROS Node Name for Logging
const char * ROS_NODE_NAME = "robomaster_teensy_node";

// Helper macro for periodic execution
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// Helper macro for checking rcl return codes during critical setup (halts on failure)
#define SETUP_RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ RCUTILS_LOG_FATAL_NAMED(ROS_NODE_NAME, "Failed critical setup on line %d, Error code: %d", __LINE__, temp_rc); error_loop();}}
// Helper macro for checking rcl return codes during entity creation (returns false on failure)
#define CREATE_RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "Failed entity creation on line %d, Error code: %d", __LINE__, temp_rc); return false;}}

// Error loop for micro-ROS critical failures
void error_loop(){
  RCUTILS_LOG_FATAL_NAMED(ROS_NODE_NAME, "Critical micro-ROS error. Halting system.");
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED rapidly
    delay(100);
  }
}

// Callback function for cmd_vel subscriber
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vel_linear_x = msg->linear.x;
  cmd_vel_linear_y = msg->linear.y;
  cmd_vel_angular_z = msg->angular.z;
  last_cmd_time = millis(); // Update last command time
  // RCUTILS_LOG_DEBUG_NAMED(ROS_NODE_NAME, "Received cmd_vel: LX:%.2f LY:%.2f AZ:%.2f", cmd_vel_linear_x, cmd_vel_linear_y, cmd_vel_angular_z);
}

// Functions to create and destroy micro-ROS entities
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  CREATE_RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  CREATE_RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, "", &support));
  
  CREATE_RCCHECK(rclc_subscription_init_default(
    &subscriber_cmd_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "mecanum/cmd_vel")); // Topic name for subscribing to velocity commands
    
  CREATE_RCCHECK(rclc_publisher_init_default(
    &motor_speed_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor_rotate_speed")); // Topic name for publishing motor 0 speed

  // Initialize executor with 1 handle (for the subscription)
  CREATE_RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); 
  CREATE_RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_cmd_vel, &twist_msg_buffer, &cmd_vel_callback, ON_NEW_DATA));
  
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "micro-ROS entities created successfully.");
  return true;
}

void destroy_entities()
{
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "Destroying micro-ROS entities...");
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0); // Set timeout to 0 for immediate destruction

  // Clean up publisher
  if (motor_speed_publisher.impl != NULL) {
      rcl_ret_t ret_pub = rcl_publisher_fini(&motor_speed_publisher, &node);
      if (ret_pub != RCL_RET_OK) RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "Failed to destroy motor_speed_publisher. Error: %d", ret_pub);
  }
  // Clean up subscriber
  if (subscriber_cmd_vel.impl != NULL) { 
      rcl_ret_t ret_sub = rcl_subscription_fini(&subscriber_cmd_vel, &node);
      if (ret_sub != RCL_RET_OK) RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "Failed to destroy subscriber_cmd_vel. Error: %d", ret_sub);
  }
  // Clean up executor
  rcl_ret_t ret_exec = rclc_executor_fini(&executor); // Executor should be cleaned up before node and support
  if (ret_exec != RCL_RET_OK) RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "Failed to destroy executor. Error: %d", ret_exec);
  
  // Clean up node
  if (node.impl != NULL) { 
      rcl_ret_t ret_node = rcl_node_fini(&node);
      if (ret_node != RCL_RET_OK) RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "Failed to destroy node. Error: %d", ret_node);
  }
  // Clean up support object
  rcl_ret_t ret_supp = rclc_support_fini(&support);
  if (ret_supp != RCL_RET_OK) RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "Failed to destroy support. Error: %d", ret_supp);
  
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "micro-ROS entities destroyed.");
}

void setup() {
  // Serial.begin(115200); // micro_ros_arduino handles Serial init if used for transport.
                         // If using Serial for debug AND micro-ROS, ensure they use different ports or are managed.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  set_microros_transports(); // Configures transport (often Serial) and initializes it.
                             // Ensure this matches your hardware setup (e.g., Serial, SPI, WiFi).
  delay(2000); // Wait for transport to be ready

  state = WAITING_AGENT;
  
  initializeCAN(&CAN1); 
  
  last_cmd_time = millis();
  last_control_time = millis();
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "RoboMaster Teensy Controller Initialized (v0.6). Waiting for micro-ROS Agent...");
}

void initializeCAN(MCP_CAN* can) {
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "Initializing CAN Bus controller (MCP2515)...");
  char point_buffer[2] = "."; 

  // Attempt to initialize CAN communication
  while (CAN_OK != can->begin(MCP_ANY, MCP_CAN_KBPS, MCP_CAN_SPI_MHZ)) {
    RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, point_buffer); // Prints "." on a new line each time.
    delay(CAN_INIT_RETRY_DELAY_MS);
  }
  can->setMode(MCP_NORMAL); // Set to normal mode
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "CAN Bus controller initialization Success!");
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      // Periodically ping the agent to check availability
      EXECUTE_EVERY_N_MS(AGENT_PING_INTERVAL_MS_WAITING, 
        state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED slowly
      delay(250); 
      break;

    case AGENT_AVAILABLE:
      RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "State: AGENT_AVAILABLE - Attempting to create micro-ROS entities...");
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) { // If entity creation failed
        RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "Failed to create entities. Returning to WAITING_AGENT state.");
        destroy_entities(); // Clean up partially created entities
      };
      break;

    case AGENT_CONNECTED:
      digitalWrite(LED_PIN, HIGH); // LED solid ON when connected
      // Periodically ping the agent to ensure connection is alive
      EXECUTE_EVERY_N_MS(AGENT_PING_INTERVAL_MS_CONNECTED, 
        state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      
      if (state == AGENT_CONNECTED) {
        // Spin the executor to process incoming messages and timers
        rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, EXECUTOR_SPIN_TIMEOUT_NS); 
        if (spin_ret != RCL_RET_OK && spin_ret != RCL_RET_TIMEOUT) {
          RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "Executor spin failed. Error code: %d", spin_ret);
        }

        // Receive motor feedback from CAN bus
        receiveFeedback(); 
        
        // Control loop execution at a fixed rate
        if (millis() - last_control_time >= CONTROL_LOOP_PERIOD_MS) { 
          calculateTargetValues(); // Calculate target RPM based on cmd_vel
          velocityControl();       // Run PID velocity control
          sendMotorCommands();     // Send new current commands to motors

          // Publish motor 0's current speed
          motor_speed_msg.data = current_velocity[0]; 
          rcl_ret_t pub_ret = rcl_publish(&motor_speed_publisher, &motor_speed_msg, NULL);
          if (pub_ret != RCL_RET_OK) { 
            RCUTILS_LOG_WARN_NAMED(ROS_NODE_NAME, "Failed to publish motor speed. Error: %d", pub_ret); 
          }

          last_control_time = millis();
        }
        
        // Timeout for cmd_vel: if no new command received, stop motors
        if (millis() - last_cmd_time > CMD_VEL_TIMEOUT_MS) { 
          cmd_vel_linear_x = 0.0;  
          cmd_vel_linear_y = 0.0;
          cmd_vel_angular_z = 0.0;
          // RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "cmd_vel timeout. Stopping motors."); // Optional: Log timeout
          stopMotors(); 
        }
      }
      break;

    case AGENT_DISCONNECTED:
      RCUTILS_LOG_WARN_NAMED(ROS_NODE_NAME, "State: AGENT_DISCONNECTED - Agent lost. Destroying entities.");
      digitalWrite(LED_PIN, LOW); // LED OFF when disconnected
      destroy_entities();
      stopMotors(); // Ensure motors are stopped
      state = WAITING_AGENT;
      break;

    default: 
      RCUTILS_LOG_ERROR_NAMED(ROS_NODE_NAME, "Error: Unknown state! Reverting to WAITING_AGENT.");
      state = WAITING_AGENT; // Fallback to a known state
      break;
  }
  
  processSerialInput(); // Handle any debug commands from Serial
}

// Checks for motor feedback messages on the specified CAN bus
void checkMotorFeedbackFromBus(MCP_CAN* can) {
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long id = 0; // CAN ID of the received message
  
  // Check if a message is available on the CAN bus
  while (CAN_MSGAVAIL == can->checkReceive()) { 
    can->readMsgBuf(&id, &len, buf); // Read the message
    
    // Check if the ID matches the expected range for motor feedback
    // Motor IDs are typically CAN_ID_RX_BASE + 1, CAN_ID_RX_BASE + 2, ...
    if (id >= (CAN_ID_RX_BASE + 1) && id <= (CAN_ID_RX_BASE + MOTOR_COUNT)) {
      int motor_idx = id - (CAN_ID_RX_BASE + 1); // Determine motor index (0 to MOTOR_COUNT-1)
      
      if (motor_idx >= 0 && motor_idx < MOTOR_COUNT) { 
        int16_t raw_angle = (buf[0] << 8) | buf[1]; // Bytes 0,1: Angle
        processAngle(motor_idx, raw_angle);         // Process raw angle for multi-turn tracking
        current_velocity[motor_idx] = (buf[2] << 8) | buf[3]; // Bytes 2,3: Velocity (RPM)
        current_torque[motor_idx] = (buf[4] << 8) | buf[5];   // Bytes 4,5: Torque current
        current_temperature[motor_idx] = buf[6];              // Byte 6: Temperature (°C)
      }
    }
  }
}

// Main function to receive feedback from all configured CAN buses
void receiveFeedback() {
  checkMotorFeedbackFromBus(&CAN1); // Check the primary CAN bus
  // If more CAN buses, call checkMotorFeedbackFromBus for them here
}

// Processes raw encoder angle to track continuous multi-turn angle
void processAngle(int motor_idx, int16_t raw_angle) {
  // Initialize on first call or if reset
  if (last_angle[motor_idx] == 0 && multi_angle[motor_idx] == 0) { 
    last_angle[motor_idx] = raw_angle;
    current_angle[motor_idx] = raw_angle;
    multi_angle[motor_idx] = raw_angle;
    return;
  }
  
  angle_difference[motor_idx] = raw_angle - last_angle[motor_idx];
  
  // Handle encoder wrap-around (e.g., 0 to 8191 range)
  if (angle_difference[motor_idx] > ENCODER_HALF_VALUE) { // Wrapped from high to low
    angle_difference[motor_idx] -= ENCODER_FULL_RANGE;
  } else if (angle_difference[motor_idx] < -ENCODER_HALF_VALUE) { // Wrapped from low to high
    angle_difference[motor_idx] += ENCODER_FULL_RANGE;
  }
  
  multi_angle[motor_idx] += angle_difference[motor_idx]; // Accumulate difference for continuous angle
  current_angle[motor_idx] = raw_angle;                  // Store current raw angle
  last_angle[motor_idx] = raw_angle;                     // Update last angle for next calculation
}

// Calculates target motor velocities (RPM) based on cmd_vel and mecanum kinematics
void calculateTargetValues() {
  float vx = cmd_vel_linear_x;    // Forward/backward m/s
  float vy = cmd_vel_linear_y;    // Strafing m/s
  float omega = cmd_vel_angular_z; // Rotational rad/s
  
  float motor_surface_speed_mps[MOTOR_COUNT]; // Wheel surface speed in m/s
  // Effective distance for rotation calculation (lx + ly for mecanum)
  float lxy_sum = (ROBOT_WIDTH / 2.0f) + (ROBOT_LENGTH / 2.0f);

  // Mecanum inverse kinematics (assuming standard X-configuration)
  // Motor 0: Front-Left
  // Motor 1: Front-Right
  // Motor 2: Rear-Left
  // Motor 3: Rear-Right
  motor_surface_speed_mps[0] = vx - vy - omega * lxy_sum; 
  motor_surface_speed_mps[1] = vx + vy + omega * lxy_sum; 
  motor_surface_speed_mps[2] = vx + vy - omega * lxy_sum; 
  motor_surface_speed_mps[3] = vx - vy + omega * lxy_sum; 

  for (int i = 0; i < MOTOR_COUNT; i++) {
    // Convert wheel surface speed (m/s) to wheel angular velocity (rad/s)
    float wheel_angular_velocity_radps = motor_surface_speed_mps[i] / WHEEL_RADIUS;
    // Convert wheel angular velocity (rad/s) to wheel RPM
    float wheel_rpm = wheel_angular_velocity_radps * (60.0f / (2.0f * PI)); // PI is from Arduino.h
    // Convert wheel RPM to motor RPM using gear ratio
    target_velocity[i] = wheel_rpm * GEAR_RATIO; 
    
    // Clamp target velocity to motor's maximum RPM
    if (abs(target_velocity[i]) > MAX_VELOCITY) {
        target_velocity[i] = (target_velocity[i] > 0 ? MAX_VELOCITY : -MAX_VELOCITY);
    }
  }
}

// PID Velocity Control for each motor
void velocityControl() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // Calculate error
    velocity_error[i] = target_velocity[i] - current_velocity[i];
    
    // Calculate integral term with anti-windup
    velocity_error_sum[i] += velocity_error[i] * CONTROL_LOOP_PERIOD_S; 
    velocity_error_sum[i] = constrain(velocity_error_sum[i], -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    
    // Calculate derivative term
    float derivative = (velocity_error[i] - velocity_error_prev[i]) / CONTROL_LOOP_PERIOD_S; 
    
    // Calculate PID output (current command for C620)
    float output = velocity_kp[i] * velocity_error[i] + 
                   velocity_ki[i] * velocity_error_sum[i] + 
                   velocity_kd[i] * derivative;
    
    // Clamp output to C620's max current range
    output = constrain(output, -MAX_CURRENT, MAX_CURRENT); 
    setMotorOutput(i, (int16_t)output); // Store the calculated current command
    
    // Save current error for next iteration's derivative calculation
    velocity_error_prev[i] = velocity_error[i];
  }
}

// Stores the calculated current command for each motor
int16_t motor_output_current[MOTOR_COUNT] = {0}; // Array to hold current commands for CAN transmission
void setMotorOutput(int motor_idx, int16_t current_command) {
  if (motor_idx >= 0 && motor_idx < MOTOR_COUNT) {
    motor_output_current[motor_idx] = current_command;
  }
}

// Stops all motors by setting their target current and velocity to zero
void stopMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motor_output_current[i] = 0; // Set output current to 0
    target_velocity[i] = 0;      // Set target velocity to 0 for PID
  }
  sendMotorCommands(); // Send the zero current commands immediately
  // RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "All motors stopped."); // Optional: Log motor stop
}

// Sends motor current commands over CAN bus
void sendMotorCommands() {
  unsigned char data[8]; // 8-byte data payload for CAN message
  
  // Pack current commands for 4 motors into the data array
  // Each command is int16_t, so 2 bytes per motor
  data[0] = (motor_output_current[0] >> 8) & 0xFF; // Motor 0 High Byte
  data[1] = motor_output_current[0] & 0xFF;       // Motor 0 Low Byte
  data[2] = (motor_output_current[1] >> 8) & 0xFF; // Motor 1 High Byte
  data[3] = motor_output_current[1] & 0xFF;       // Motor 1 Low Byte
  data[4] = (motor_output_current[2] >> 8) & 0xFF; // Motor 2 High Byte
  data[5] = motor_output_current[2] & 0xFF;       // Motor 2 Low Byte
  data[6] = (motor_output_current[3] >> 8) & 0xFF; // Motor 3 High Byte
  data[7] = motor_output_current[3] & 0xFF;       // Motor 3 Low Byte
  
  // Send the CAN message with CAN_ID_TX (0x200 for C620)
  // Standard CAN frame, 8 data bytes
  CAN1.sendMsgBuf(CAN_ID_TX, 0, 8, data); 
}

// Processes simple commands received over Serial for debugging
void processSerialInput() {
  if (Serial.available() > 0) { // Check if Serial data is available (for USB Serial on Teensy)
    char command = Serial.read();
    
    switch (command) {
      case 's': // Stop motors
        cmd_vel_linear_x = 0.0; 
        cmd_vel_linear_y = 0.0;
        cmd_vel_angular_z = 0.0;
        stopMotors();
        RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "Motors stopped (Serial command 's').");
        break;
      case 'd': // Display motor status
        printMotorStatus();
        break;
      // Add more debug commands as needed
    }
  }
}

// Prints detailed status of motors and robot state to logger
void printMotorStatus() {
  char buffer[256]; // Buffer for formatting log messages
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "---------- Motor Status (Velocity Mode Only) ----------");
  for (int i = 0; i < MOTOR_COUNT; i++) {
    snprintf(buffer, sizeof(buffer), 
             "M%d: Angle(raw)=%d, Angle(multi)=%ld, Vel(act)=%d RPM (tgt=%.1f RPM), Torque=%d, Temp=%dC, OutCur=%d",
             i, current_angle[i], multi_angle[i], current_velocity[i], target_velocity[i],
             current_torque[i], current_temperature[i], motor_output_current[i]);
    RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, buffer);
  }
  snprintf(buffer, sizeof(buffer), "CmdVel: X=%.2f m/s, Y=%.2f m/s, Z=%.2f rad/s", cmd_vel_linear_x, cmd_vel_linear_y, cmd_vel_angular_z);
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, buffer);
  
  const char* currentStateStr = "UNKNOWN";
  switch(state){
    case WAITING_AGENT: currentStateStr = "WAITING_AGENT"; break;
    case AGENT_AVAILABLE: currentStateStr = "AGENT_AVAILABLE"; break;
    case AGENT_CONNECTED: currentStateStr = "AGENT_CONNECTED"; break;
    case AGENT_DISCONNECTED: currentStateStr = "AGENT_DISCONNECTED"; break;
  }
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "ROS Agent State: %s", currentStateStr);
  RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME, "----------------------------------------------------");
}
