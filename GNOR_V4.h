/*
 * boat.hpp
 * Platform-specific definitions for boat control system
 */

#ifndef BOAT_HPP
#define BOAT_HPP

#include <Arduino.h>

#define USE_BOAT          // Include boat loop
//#define DUAL_MOTOR      // differential steering: left=servo2, right=servoEsc, no rudder
//#define OPEN_LOOP         // Disregard sensed headings and hard code trajectory

#define USE_MPU           // Comment out to disable MPU6050 (must enable to use closed loop)
#define USE_LEDS          // Comment out to disable LEDs
#define USE_EXT_PULLUPS   // Include external 4.7k pull-up resistors on I2C lines

//#define USE_SERVO_TEST  // Comment out to disable servo sweep test
//#define USE_LED_TEST    // Comment out to disable LED cycle test (requires USE_LEDS)

// Platform-specific pin definitions
#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO)
    // Arduino Nano / Uno
    // Servo library uses timer-based PWM and works on any digital pin,
    // but pins 9 and 10 (Timer1) are preferred for servo use.
    // Avoid pins 5 and 6 if precise timing matters (Timer0, shared with millis).
    #define SERVO1_PIN       9   // Rudder
    #define SERVO2_PIN       10  // Right motor (dual motor config)
    #define SERVO3_PIN       3   // Auxiliary servo
    #define ESC_PIN          11  // Single motor OR left motor (dual motor config)
    #define MOTOR_SWITCH     4   // Active low — use INPUT_PULLUP
    #define CALIBRATE_SWITCH 7   // Active low — use INPUT_PULLUP
    #define STARTUP_LED_PIN  13  // Onboard LED to indicate startup mode
    #define DRIFT_LED_PIN    5   // LED to indicate IMU drift
    #define HEADING_LED_PIN  6   // LED to indicate heading
    #define INT_PIN          2   // Reserved for potential MPU6050 INT wiring (not currently used)
#else
    #error "Unsupported platform. Please define Arduino Nano or Uno."
#endif

// Function declarations
bool motorSwitchPressed();
bool calibrateSwitchPressed();
bool escSwitchPressed();

#endif // BOAT_HPP
