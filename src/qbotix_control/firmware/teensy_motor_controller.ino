/**
 * QBotix Rover Motor Controller - Teensy 4.1 with micro-ROS
 * 
 * Motor Driver: MDS40B Cytron (Anti Locked Phase mode)
 * - PWM > 50% = Forward
 * - PWM < 50% = Reverse  
 * - PWM = 50% = Stop
 * 
 * 6-Wheel Skid Steering Configuration:
 * Right Side: R1(18), R2(23), R3(16)
 * Left Side: L1(12), L2(27), L3(26)
 * 
 * micro-ROS:
 * Subscribes: /cmd_vel (geometry_msgs/Twist)
 * Publishes:  /wheel_rpm (std_msgs/Float32MultiArray) [left_rpm, right_rpm]
 * 
 * PWM Resolution: 12-bit (0-4095) at 20kHz for smooth motor control
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
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

// LED for status
#define LED_PIN 13

// ============ ROBOT PARAMETERS ============
#define WHEEL_RADIUS 0.10f        // meters
#define WHEEL_SEPARATION 0.53f    // meters (track width)
#define MAX_RPM 150.0f            // Maximum motor RPM
#define MAX_LINEAR_VEL 1.57f      // m/s (from 150 RPM)
#define MAX_ANGULAR_VEL 1.57f     // rad/s

// ============ PWM PARAMETERS ============
#define PWM_RESOLUTION 12         // 12-bit PWM (0-4095)
#define PWM_FREQUENCY 20000       // 20kHz PWM frequency
#define PWM_CENTER 2048           // 50% duty cycle (stop)
#define PWM_RANGE 2047            // Range from center to max/min

// ============ SAFETY PARAMETERS ============
#define CMD_TIMEOUT_MS 500        // Stop motors if no command for 500ms
#define RAMP_RATE 100.0f          // RPM per second for smooth acceleration
#define CONTROL_RATE_MS 20        // Motor update rate (50Hz)
#define PUBLISH_RATE_MS 100       // Publish feedback at 10Hz

// ============ MICRO-ROS OBJECTS ============
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t wheel_rpm_pub;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Float32MultiArray wheel_rpm_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

// ============ GLOBAL VARIABLES ============
float target_left_rpm = 0.0f;
float target_right_rpm = 0.0f;
float current_left_rpm = 0.0f;
float current_right_rpm = 0.0f;

unsigned long last_cmd_time = 0;
unsigned long last_publish_time = 0;
bool micro_ros_connected = false;

// Pre-allocate array for wheel_rpm_msg
float wheel_rpm_data[2] = {0.0f, 0.0f};

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
    
    emergencyStop();
}

int rpmToPWM(float rpm) {
    rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
    float normalized = rpm / MAX_RPM;
    int pwm_offset = (int)(normalized * PWM_RANGE);
    return constrain(PWM_CENTER + pwm_offset, 0, 4095);
}

void setMotorPWM(int pin, float rpm) {
    analogWrite(pin, rpmToPWM(rpm));
}

void emergencyStop() {
    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;
    current_left_rpm = 0.0f;
    current_right_rpm = 0.0f;
    
    analogWrite(L1_PWM, PWM_CENTER);
    analogWrite(L2_PWM, PWM_CENTER);
    analogWrite(L3_PWM, PWM_CENTER);
    analogWrite(R1_PWM, PWM_CENTER);
    analogWrite(R2_PWM, PWM_CENTER);
    analogWrite(R3_PWM, PWM_CENTER);
}

float rampValue(float current, float target, float max_change) {
    float diff = target - current;
    if (fabs(diff) <= max_change) return target;
    return current + (diff > 0 ? max_change : -max_change);
}

void updateMotors() {
    // Calculate max RPM change per control cycle
    float max_rpm_change = RAMP_RATE * (CONTROL_RATE_MS / 1000.0f);
    
    // Smooth acceleration
    current_left_rpm = rampValue(current_left_rpm, target_left_rpm, max_rpm_change);
    current_right_rpm = rampValue(current_right_rpm, target_right_rpm, max_rpm_change);
    
    // Set left side motors
    setMotorPWM(L1_PWM, current_left_rpm);
    setMotorPWM(L2_PWM, current_left_rpm);
    setMotorPWM(L3_PWM, current_left_rpm);
    
    // Set right side motors
    setMotorPWM(R1_PWM, current_right_rpm);
    setMotorPWM(R2_PWM, current_right_rpm);
    setMotorPWM(R3_PWM, current_right_rpm);
}

// ============ KINEMATICS ============
void cmdVelToWheelRPM(float linear_x, float angular_z) {
    // Clamp input velocities
    linear_x = constrain(linear_x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    angular_z = constrain(angular_z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    
    // Differential drive kinematics for skid steering
    // v_left = v - w * L/2
    // v_right = v + w * L/2
    float left_vel = linear_x - (angular_z * WHEEL_SEPARATION / 2.0f);
    float right_vel = linear_x + (angular_z * WHEEL_SEPARATION / 2.0f);
    
    // Convert velocity (m/s) to RPM: RPM = v * 60 / (2 * pi * r)
    float vel_to_rpm = 60.0f / (2.0f * PI * WHEEL_RADIUS);
    float left_rpm = left_vel * vel_to_rpm;
    float right_rpm = right_vel * vel_to_rpm;
    
    // Scale if exceeds max RPM
    float max_rpm_abs = max(fabs(left_rpm), fabs(right_rpm));
    if (max_rpm_abs > MAX_RPM) {
        float scale = MAX_RPM / max_rpm_abs;
        left_rpm *= scale;
        right_rpm *= scale;
    }
    
    target_left_rpm = left_rpm;
    target_right_rpm = right_rpm;
}

// ============ ROS CALLBACKS ============
void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    
    // Convert cmd_vel to wheel RPMs
    cmdVelToWheelRPM(msg->linear.x, msg->angular.z);
    
    last_cmd_time = millis();
    digitalWrite(LED_PIN, HIGH);  // LED on when receiving commands
}

void control_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Check command timeout
        if ((millis() - last_cmd_time) > CMD_TIMEOUT_MS) {
            target_left_rpm = 0.0f;
            target_right_rpm = 0.0f;
            digitalWrite(LED_PIN, LOW);  // LED off when timed out
        }
        
        // Update motors
        updateMotors();
        
        // Publish wheel RPM feedback
        if ((millis() - last_publish_time) >= PUBLISH_RATE_MS) {
            wheel_rpm_data[0] = current_left_rpm;
            wheel_rpm_data[1] = current_right_rpm;
            wheel_rpm_msg.data.data = wheel_rpm_data;
            wheel_rpm_msg.data.size = 2;
            
            RCSOFTCHECK(rcl_publish(&wheel_rpm_pub, &wheel_rpm_msg, NULL));
            last_publish_time = millis();
        }
    }
}

// ============ MICRO-ROS SETUP ============
void setupMicroROS() {
    set_microros_transports();
    
    allocator = rcl_get_default_allocator();
    
    // Create init_options and support
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // Create node: /teensy_motor_controller
    RCCHECK(rclc_node_init_default(&node, "teensy_motor_controller", "", &support));
    
    // Create subscriber: /cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    
    // Create publisher: /wheel_rpm
    RCCHECK(rclc_publisher_init_default(
        &wheel_rpm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_rpm"));
    
    // Initialize wheel_rpm_msg
    wheel_rpm_msg.data.capacity = 2;
    wheel_rpm_msg.data.size = 2;
    wheel_rpm_msg.data.data = wheel_rpm_data;
    
    // Create timer for control loop (50Hz = 20ms)
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(CONTROL_RATE_MS),
        control_timer_callback));
    
    // Create executor with 2 handles (subscriber + timer)
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, 
            &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    
    micro_ros_connected = true;
}

// ============ MAIN SETUP ============
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    // Setup motors first
    setupMotors();
    
    // Wait a bit for USB enumeration
    delay(2000);
    
    // Setup micro-ROS
    setupMicroROS();
    
    last_cmd_time = millis();
    last_publish_time = millis();
    
    digitalWrite(LED_PIN, LOW);  // LED off = ready
}

// ============ MAIN LOOP ============
void loop() {
    if (micro_ros_connected) {
        // Spin executor - handles callbacks
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    } else {
        // Safety: keep motors stopped if not connected
        emergencyStop();
        
        // Blink LED to show disconnected
        digitalWrite(LED_PIN, (millis() / 500) % 2);
    }
}
