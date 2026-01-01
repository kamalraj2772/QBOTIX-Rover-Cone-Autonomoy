/**
 * QBotix Rover Motor Controller - Teensy 4.1 with micro-ROS
 * 
 * Uses micro_ros_arduino to communicate directly with ROS2
 * Motor Driver: MDS40B Cytron (Anti Locked Phase mode)
 * 
 * Subscribes to: /cmd_vel (geometry_msgs/Twist)
 * Publishes to:  /motor_status (std_msgs/String)
 *                /wheel_odom (nav_msgs/Odometry) - optional wheel feedback
 * 
 * 6-Wheel Skid Steering Configuration:
 * Right Side: R1(18), R2(23), R3(16)
 * Left Side: L1(12), L2(27), L3(26)
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

// ============ PIN DEFINITIONS ============
// Right Side Motors
#define R1_PWM 18
#define R2_PWM 23
#define R3_PWM 16

// Left Side Motors
#define L1_PWM 12
#define L2_PWM 27
#define L3_PWM 26

// Status LED
#define LED_PIN 13

// ============ ROBOT PARAMETERS ============
#define WHEEL_RADIUS 0.10f        // meters
#define WHEEL_SEPARATION 0.53f    // meters (track width)
#define MAX_RPM 150.0f            // Maximum motor RPM
#define MAX_LINEAR_VEL 1.57f      // m/s
#define MAX_ANGULAR_VEL 1.57f     // rad/s

// ============ PWM PARAMETERS ============
#define PWM_RESOLUTION 12         // 12-bit PWM (0-4095)
#define PWM_FREQUENCY 20000       // 20kHz PWM frequency
#define PWM_CENTER 2048           // 50% duty cycle (stop)
#define PWM_RANGE 2047            // Range from center to max/min

// ============ SAFETY PARAMETERS ============
#define CMD_TIMEOUT_MS 500        // Stop motors if no command for 500ms
#define RAMP_RATE 100.0f          // RPM per second for acceleration
#define CONTROL_RATE_HZ 50        // Control loop rate

// ============ MICRO-ROS OBJECTS ============
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t motor_rpm_publisher;

geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32MultiArray motor_rpm_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ============ GLOBAL VARIABLES ============
float target_left_rpm = 0.0f;
float target_right_rpm = 0.0f;
float current_left_rpm = 0.0f;
float current_right_rpm = 0.0f;

unsigned long last_cmd_time = 0;
unsigned long last_control_time = 0;

bool micro_ros_init_successful = false;

// ============ ERROR HANDLING ============
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
    while(1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

// ============ MOTOR FUNCTIONS ============
void setupMotors() {
    // Configure PWM resolution and frequency
    analogWriteResolution(PWM_RESOLUTION);
    
    analogWriteFrequency(R1_PWM, PWM_FREQUENCY);
    analogWriteFrequency(R2_PWM, PWM_FREQUENCY);
    analogWriteFrequency(R3_PWM, PWM_FREQUENCY);
    analogWriteFrequency(L1_PWM, PWM_FREQUENCY);
    analogWriteFrequency(L2_PWM, PWM_FREQUENCY);
    analogWriteFrequency(L3_PWM, PWM_FREQUENCY);
    
    pinMode(R1_PWM, OUTPUT);
    pinMode(R2_PWM, OUTPUT);
    pinMode(R3_PWM, OUTPUT);
    pinMode(L1_PWM, OUTPUT);
    pinMode(L2_PWM, OUTPUT);
    pinMode(L3_PWM, OUTPUT);
    
    // Initialize to stop
    stopMotors();
}

int rpmToPWM(float rpm) {
    // Clamp RPM to valid range
    rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
    
    // Convert RPM to PWM value for MDS40B Anti-Locked Phase
    // PWM = 2048 (50%) = Stop
    // PWM > 2048 = Forward
    // PWM < 2048 = Reverse
    float normalized = rpm / MAX_RPM;
    int pwm_offset = (int)(normalized * PWM_RANGE);
    int pwm_value = PWM_CENTER + pwm_offset;
    
    return constrain(pwm_value, 0, 4095);
}

void setMotorPWM(int pin, float rpm) {
    analogWrite(pin, rpmToPWM(rpm));
}

void stopMotors() {
    analogWrite(L1_PWM, PWM_CENTER);
    analogWrite(L2_PWM, PWM_CENTER);
    analogWrite(L3_PWM, PWM_CENTER);
    analogWrite(R1_PWM, PWM_CENTER);
    analogWrite(R2_PWM, PWM_CENTER);
    analogWrite(R3_PWM, PWM_CENTER);
    
    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;
    current_left_rpm = 0.0f;
    current_right_rpm = 0.0f;
}

float rampValue(float current, float target, float dt) {
    float max_change = RAMP_RATE * dt;
    float diff = target - current;
    
    if (fabs(diff) <= max_change) {
        return target;
    }
    
    return current + (diff > 0 ? max_change : -max_change);
}

void updateMotors(float dt) {
    // Smooth acceleration
    current_left_rpm = rampValue(current_left_rpm, target_left_rpm, dt);
    current_right_rpm = rampValue(current_right_rpm, target_right_rpm, dt);
    
    // Set all left side motors
    setMotorPWM(L1_PWM, current_left_rpm);
    setMotorPWM(L2_PWM, current_left_rpm);
    setMotorPWM(L3_PWM, current_left_rpm);
    
    // Set all right side motors
    setMotorPWM(R1_PWM, current_right_rpm);
    setMotorPWM(R2_PWM, current_right_rpm);
    setMotorPWM(R3_PWM, current_right_rpm);
}

// ============ KINEMATICS ============
void cmdVelToWheelRPM(float linear_vel, float angular_vel, float* left_rpm, float* right_rpm) {
    // Clamp velocities
    linear_vel = constrain(linear_vel, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    angular_vel = constrain(angular_vel, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    
    // Differential drive kinematics
    // v_left = v - w * L/2
    // v_right = v + w * L/2
    float left_vel = linear_vel - (angular_vel * WHEEL_SEPARATION / 2.0f);
    float right_vel = linear_vel + (angular_vel * WHEEL_SEPARATION / 2.0f);
    
    // Convert velocity (m/s) to RPM
    // RPM = v * 60 / (2 * pi * r)
    float vel_to_rpm = 60.0f / (2.0f * PI * WHEEL_RADIUS);
    
    *left_rpm = left_vel * vel_to_rpm;
    *right_rpm = right_vel * vel_to_rpm;
    
    // Scale if exceeds max RPM
    float max_rpm_val = max(fabs(*left_rpm), fabs(*right_rpm));
    if (max_rpm_val > MAX_RPM) {
        float scale = MAX_RPM / max_rpm_val;
        *left_rpm *= scale;
        *right_rpm *= scale;
    }
}

// ============ ROS CALLBACKS ============
void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    
    // Convert cmd_vel to wheel RPMs
    cmdVelToWheelRPM(msg->linear.x, msg->angular.z, &target_left_rpm, &target_right_rpm);
    
    last_cmd_time = millis();
    
    // LED feedback - blink on command
    digitalWrite(LED_PIN, HIGH);
}

void publishMotorRPM() {
    // Publish current motor RPMs [left, right]
    motor_rpm_msg.data.data[0] = (int32_t)current_left_rpm;
    motor_rpm_msg.data.data[1] = (int32_t)current_right_rpm;
    motor_rpm_msg.data.size = 2;
    
    RCSOFTCHECK(rcl_publish(&motor_rpm_publisher, &motor_rpm_msg, NULL));
}

// ============ MICRO-ROS SETUP ============
bool setupMicroROS() {
    // Set micro-ROS transport (USB Serial)
    set_microros_transports();
    
    delay(2000);  // Wait for agent connection
    
    allocator = rcl_get_default_allocator();
    
    // Create init_options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    
    // Create support
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    // Create node
    RCCHECK(rclc_node_init_default(&node, "teensy_motor_controller", "", &support));
    
    // Create cmd_vel subscriber
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    
    // Create motor_rpm publisher
    RCCHECK(rclc_publisher_init_default(
        &motor_rpm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "motor_rpm_feedback"));
    
    // Initialize motor_rpm_msg
    motor_rpm_msg.data.capacity = 2;
    motor_rpm_msg.data.size = 2;
    motor_rpm_msg.data.data = (int32_t*)malloc(2 * sizeof(int32_t));
    motor_rpm_msg.data.data[0] = 0;
    motor_rpm_msg.data.data[1] = 0;
    
    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, 
            &cmd_vel_callback, ON_NEW_DATA));
    
    return true;
}

// ============ MAIN SETUP ============
void setup() {
    // Setup LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    // Setup motors
    setupMotors();
    
    // Setup micro-ROS
    micro_ros_init_successful = setupMicroROS();
    
    if (micro_ros_init_successful) {
        digitalWrite(LED_PIN, LOW);  // LED off = connected
    }
    
    last_control_time = millis();
}

// ============ MAIN LOOP ============
void loop() {
    if (micro_ros_init_successful) {
        // Spin executor (non-blocking)
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
        
        // Control loop at fixed rate
        unsigned long now = millis();
        float dt = (now - last_control_time) / 1000.0f;
        
        if (dt >= (1.0f / CONTROL_RATE_HZ)) {
            // Check command timeout
            if ((now - last_cmd_time) > CMD_TIMEOUT_MS) {
                target_left_rpm = 0.0f;
                target_right_rpm = 0.0f;
                digitalWrite(LED_PIN, LOW);  // LED off = no commands
            }
            
            // Update motors with ramping
            updateMotors(dt);
            
            // Publish motor RPM feedback
            publishMotorRPM();
            
            last_control_time = now;
        }
    } else {
        // Not connected - try to reconnect periodically
        static unsigned long last_reconnect = 0;
        if (millis() - last_reconnect > 5000) {
            last_reconnect = millis();
            micro_ros_init_successful = setupMicroROS();
        }
        
        // Safety - keep motors stopped
        stopMotors();
        
        // Blink LED to indicate not connected
        digitalWrite(LED_PIN, (millis() / 500) % 2);
    }
}
