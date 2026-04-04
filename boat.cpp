/*
 * boat.cpp
 *
 * Servo assignments:
 *   servo1   — Rudder
 *   servo2   — Left motor  (dual motor config)
 *   servoEsc — Single motor (single motor config) OR right motor (dual motor config)
 *   servo3   — Auxiliary servo
 */

#include "GNOR_V4.h"
#include <Arduino.h>
#include <Servo.h>


extern Servo servo1;    // Rudder — declared in GNOR_V4.ino
extern Servo servo2;    // Right motor (dual motor)
extern Servo servo3;    // Auxiliary servo
extern Servo servoEsc;  // Single motor OR left motor (dual motor)

unsigned long last_time = 0;     // last time through the loop

#define LED2_DEAD_BAND      5.0f    // degrees — within this shows bright on-target green
#define LED2_MAX_ERROR      45.0f   // degrees — clamp for brightness gradient
#define LED2_MAX_BRIGHTNESS 100     // max channel brightness in the gradient zone (0–255)
#define LED2_ON_BRIGHTNESS  255     // green brightness when within LED2_DEAD_BAND

#define P 2.0                       // Proportional constant used by the rudder or for each dual motor
#define MOTOR_BASE_SPEED 0.5        // Default speed the single or dual motor(s) use (0.0-1.0)
#define COUNTDOWN_TIME 10           // (open loop only) Countdown time, in seconds, to motor start after switch is activated
#define THROTTLE_HIGH_DEGREES 145   // High throttle setting in servo degrees
#define THROTTLE_LOW_DEGREES 35     // Low throttle setting in servo degrees
#define MAX_RUDDER_DEGREES 90/2     // Max angle the rudder moves on each side of zero (90). Normally 45 Degrees.

// Waypoint Declaration========================================================================================================
#ifndef OPEN_LOOP           // CLOSED LOOP-------------------------------------------------------------------------------------
struct Waypoint {
    unsigned long time_ms;  // elapsed mission time to activate this heading
    int heading360;         // compass heading in degrees (0–360)
};
#elif defined(DUAL_MOTOR)   // OPEN LOOP WITH DUAL PROP------------------------------------------------------------------------
struct Waypoint {
    unsigned long time_ms;  // elapsed mission time to activate this heading
    float percent_diff;     // 1.0 turns L full throttle, -1.0 turns R full throttle, 0 stays straight
};
#else                       // OPEN LOOP WITH RUDDER---------------------------------------------------------------------------
struct Waypoint {
    unsigned long time_ms;  // elapsed mission time to activate this heading
    int servo_angle;        // servo offset in degrees, CW positive, centered at 0 (Note: magnitude capped by MAX_RUDDER_DEGREES)
};
#endif

// Waypoint Array==============================================
// CLOSED LOOP-------------------------------------------------
#ifndef OPEN_LOOP
static const Waypoint waypoints[] = {
    {     0,   0 },   // 0–10s:  straight ahead
    { 10000, 270 },   // 10–20s: turn to 270
    { 20000, 180 },   // 20s+:   turn to 180
};
// OPEN LOOP WITH DUAL PROP------------------------------------
#elif defined(DUAL_MOTOR)
static const Waypoint waypoints[] = {
    {     0,   0 },   // 0–10s:  drive straight
    { 10000, 0.5 },   // 10–11s: turn L at half current throttle
    { 11000,   0 },   // 11–19s: drive straight
    { 19000, 0.5 },   // 19–20s: turn L at half current throttle
    { 20000,   0 },   // 20s+:   drive straight
};
// OPEN LOOP WITH RUDDER---------------------------------------
#else
static const Waypoint waypoints[] = {
    {     0,   0 },   // 0–10s:  rudder straight
    { 10000,  45 },   // 10–11s: turn L
    { 11000,   0 },   // 11–19s: rudder straight
    { 19000,  45 },   // 19–20s: turn L
    { 20000,   0 },   // 20s+:   rudder straight
};
#endif
static const int WAYPOINT_COUNT = sizeof(waypoints) / sizeof(waypoints[0]);

/*
 * setMotor1Speed
 * --------------
 * Set the ESC motor speed. speed is in the range 0.0 (off) to 1.0 (full).
 */
void setMotor1Speed(double speed) {
    servoEsc.write((int)(THROTTLE_LOW_DEGREES + (speed * (THROTTLE_HIGH_DEGREES - THROTTLE_LOW_DEGREES))));
}

/*
 * setMotor2Speed
 * --------------
 * Set the left motor speed (dual motor config). speed is in the range 0.0 (off) to 1.0 (full).
 */
void setMotor2Speed(double speed) {
    servo2.write((int)(THROTTLE_LOW_DEGREES + (speed * (THROTTLE_HIGH_DEGREES - THROTTLE_LOW_DEGREES))));
}

/*
 * Button/Switch functions
 * Buttons are active low (pressed = 0)
 */
bool motorSwitchPressed() {
    return digitalRead(MOTOR_SWITCH) == 0;
}

bool calibrateSwitchPressed() {
    return digitalRead(CALIBRATE_SWITCH) == 0;
}

/*
 * calculateDifferenceBetweenAngles
 * ---------------------------------
 * Return the signed difference between two angles in the -180 to +180 system.
 * Result is in the range (-180, +180].
 */
double calculateDifferenceBetweenAngles(double angle1, double angle2) {
    double delta = angle1 - angle2;
    if (delta >  180.0) delta -= 360.0;
    if (delta < -180.0) delta += 360.0;
    return delta;
}

/*
 * wrapTo180
 * ---------
 * Wrap any angle (degrees) into the range (-180, +180].
 */
double wrapTo180(double angle) {
    angle = fmod(angle, 360.0);
    if (angle >  180.0) angle -= 360.0;
    if (angle < -180.0) angle += 360.0;
    return angle;
}



/*
 * boatLoop
 * ----------------------------
 * This routine is called in the main loop at a rate of ~100 times/sec.
 * The current timestamp in milliseconds and the current heading (-180 to +180) is passed in.
 * Note: must use static variables if you need a persistence between calls.
 */
void boatLoop(unsigned long timestamp, double heading) {

	// Static variables.  Keep their values between calls to "boatLoop"
    static double last_heading=0.0;			// used to calculate the delta heading
    static unsigned long switch_time;       // time motor switch was last activated
    static unsigned long start_time;		// actual time the boat started.  Used as an offset to calculate elapsed time
    static int started=-1;					// has the boat started, -1=not ready, 0=ready, 1=started
    static double heading_zero_offset;		// heading offset.  Used to zero heading when button is pressed
    static boolean first_time=true;         // flag to run one time routines
    static boolean calibrate_time=true;     // Should we calibrate
    unsigned long running_time;				// elapsed time since the mission started
    static boolean motors_armed = false;         // is motor armed
    static boolean motor_switch_last = false;    // previous state of motor switch (for edge detection)
    static boolean motor_switch_init = false;    // has motor switch state been seeded
    static int waypoint_index = 0;               // current waypoint index
    
    int target360 = 0;                          // target heading in compass degrees (0-360)
    double target = 0.0;                        // target heading in -180 to +180
    double error = 0.0;                         // error between current heading and target heading
    int rudder = 0;                             // calculated rudder angle
    double diff = 0.0;                          // diff for dual motor drive
    
    double heading_rate;			            // Calculated delta between current and last reading of heading

    //--------------------------------------------------------------------------------
    // pre-start
    //--------------------------------------------------------------------------------

    // Calibrate ESC with max and min pulse widths
    if (calibrate_time) {
        if (calibrateSwitchPressed() == 1) {
            setMotor1Speed(1.0);
        }

        while (calibrateSwitchPressed() == 1)  {} // loop with pressed waiting for ESC beeps

        setMotor1Speed(0.0);

        calibrate_time = false;
    }

    // Run one time initialization routines.
    if (first_time) {
        Serial.println("************* BOAT LOOP STARTED *************");
        servo1.write(90);    // rudder straight
        setMotor1Speed(0.0);   // motor off
#ifdef DUAL_MOTOR
        setMotor2Speed(0.0);
#endif
        first_time = false;
        motors_armed = false;
    }

    // Detect rising edge on motor switch (not pressed -> pressed).
    // Seed last state on first call so a switch held at startup is ignored.
    boolean motor_switch_now = motorSwitchPressed();
    if (!motor_switch_init) {
        motor_switch_last = motor_switch_now;
        motor_switch_init = true;
    }

    // was switch just pressed (rising edge)
    if (!motor_switch_last && motor_switch_now) {
        servo1.write(90);    // rudder straight
        setMotor1Speed(0.0);   // motor off
#ifdef DUAL_MOTOR
        setMotor2Speed(0.0);
#endif
        switch_time = timestamp;
        heading_zero_offset = heading;
        started = 0;
        waypoint_index = 0;
        motors_armed = true;
    } else if (motor_switch_now) {
        motors_armed = true;
    } else {
        motors_armed = false;
    }
        
    motor_switch_last = motor_switch_now;
#ifndef OPEN_LOOP
    // Apply heading_zero offset and wrap into -180 to +180
    heading = calculateDifferenceBetweenAngles(heading, heading_zero_offset);

    // calculate heading turn rate.  this can be used to give a measure of how fast the boat's heading is drifting when the
    // boat is still.  If good, stable rate should be less than .005.
    heading_rate = calculateDifferenceBetweenAngles(heading, last_heading);
    last_heading = heading;

    // print heading every .5 seconds
    if ((timestamp - last_time) > 500) {
        Serial.print("Heading: ");
        Serial.print(heading);
        Serial.print(", Rate: ");
        Serial.print(heading_rate,3);
        Serial.print(", Started: ");
        Serial.println(started);
        last_time = timestamp;
    }
#endif // not OPEN_LOOP

    // if the heading rate is greater than some constant then turn on the red LED
#if defined(USE_LEDS) && !defined(OPEN_LOOP)
    if (fabs(heading_rate) > .005) {
        digitalWrite(DRIFT_LED_PIN, HIGH);
    }else{
        digitalWrite(DRIFT_LED_PIN, LOW);
    }
#endif
    
    // check for boat start
#ifndef OPEN_LOOP
    // Closed loop: rotate boat 90 degrees to start motor
    if (((calculateDifferenceBetweenAngles(heading, 90)) > 0.0) && (started==0)) {
        started = 1;
        start_time = timestamp;
        Serial.println("************* Started *************");
    }
#else
    // Open loop: wait for countdown to start motor
    if(((timestamp - switch_time) >= COUNTDOWN_TIME*1000) && (started==0)){
        started = 1;
        start_time = timestamp;
        Serial.println("************* Started *************");
    }
#endif

    // handle startup "running LED"
    // blinking: pre-start (started==0), solid: running (started==1)
#ifdef USE_LEDS
    {
        static unsigned long last_blink_time = 0;
        static bool blink_state = false;

        if (started == 1) {
            digitalWrite(STARTUP_LED_PIN, HIGH);          // solid color
        } else if (started == 0) {
            if ((timestamp - last_blink_time) >= 500) {
                blink_state = !blink_state;
                last_blink_time = timestamp;
            }
            digitalWrite(STARTUP_LED_PIN, blink_state ? HIGH : LOW); // blinking color
        } else {
            digitalWrite(STARTUP_LED_PIN, LOW);           // off when not ready
        }
    }
#endif

    //--------------------------------------------------------------------------------
    // Main routine that runs after boat has started
    //--------------------------------------------------------------------------------
    if (started==1) {
        running_time = timestamp - start_time;			// calculate elapsed time
        
        // Advance through waypoints as elapsed time passes
        while (waypoint_index + 1 < WAYPOINT_COUNT &&
               running_time >= waypoints[waypoint_index + 1].time_ms) {
            waypoint_index++;
        }
#ifndef OPEN_LOOP // CLOSED LOOP-------------------------------------------------------------------
        target360 = waypoints[waypoint_index].heading360;

        // convert compass target to -180 to +180
        target = (target360 > 180) ? target360 - 360 : target360;
        
        // PID routine
        // bigger P causes boat to have more reaction to heading errors
        error = calculateDifferenceBetweenAngles(heading, target);

#ifdef DUAL_MOTOR
        //--------------------------------------------------
        // Dual motor differential steering (no rudder)
        // Servo2: right motor
        // Esc: left motor
        // NOTE: remember to detach the red power wire from the servo2 ESC
        //--------------------------------------------------
        diff = (P * error) / 180.0;      // scale error to motor-speed units
        if (diff >  MOTOR_BASE_SPEED) diff =  MOTOR_BASE_SPEED;
        if (diff < -MOTOR_BASE_SPEED) diff = -MOTOR_BASE_SPEED;
        if (motors_armed) {
            setMotor2Speed(MOTOR_BASE_SPEED + diff);   // right
            setMotor1Speed(MOTOR_BASE_SPEED - diff);   // left
        } else {
            setMotor2Speed(0.0);
            setMotor1Speed(0.0);
        }
#else
        //--------------------------------------------------
        // Single motor + rudder
        // servo1: Rudder, servoEsc: motor
        //--------------------------------------------------
        rudder = P * error;
        if (rudder >  MAX_RUDDER_DEGREES) rudder =  MAX_RUDDER_DEGREES;
        if (rudder < -MAX_RUDDER_DEGREES) rudder = -MAX_RUDDER_DEGREES;
        servo1.write(90 + rudder);
        if (motors_armed) {
            setMotor1Speed(MOTOR_BASE_SPEED);
        } else {
            setMotor1Speed(0.0);
        }
#endif
#elif defined(DUAL_MOTOR)// OPEN LOOP DUAL PROP----------------------------------------------------
        diff = MOTOR_BASE_SPEED * waypoints[waypoint_index].percent_diff; // scale percent diff to base speed
        // Cap at +-base speed, in case percent diff mistakenly exceeded +-1
        if (diff >  MOTOR_BASE_SPEED) diff =  MOTOR_BASE_SPEED;
        if (diff < -MOTOR_BASE_SPEED) diff = -MOTOR_BASE_SPEED;
        if (motors_armed) {
            setMotor2Speed(MOTOR_BASE_SPEED + diff);    // right
            setMotor1Speed(MOTOR_BASE_SPEED - diff);    // left
        } else {
            setMotor2Speed(0.0);
            setMotor1Speed(0.0);
        }
#else // OPEN LOOP RUDDER--------------------------------------------------------------------------
        rudder = waypoints[waypoint_index].servo_angle; // Retreive waypoint rudder offset
        if (rudder >  MAX_RUDDER_DEGREES) rudder =  MAX_RUDDER_DEGREES;
        if (rudder < -MAX_RUDDER_DEGREES) rudder = -MAX_RUDDER_DEGREES;
        servo1.write(90 + rudder);
        if (motors_armed) {
            setMotor1Speed(MOTOR_BASE_SPEED);
        } else {
            setMotor1Speed(0.0);
        }
#endif // ifndef OPEN_LOOP

    }

    // Heading LED: Max brightness at currently set heading, dims with offset
#if defined(USE_LEDS) && !defined(OPEN_LOOP)
    {
        double led2_err = calculateDifferenceBetweenAngles(heading, target);
        uint8_t light = 0;
        if (fabs(led2_err) < LED2_DEAD_BAND) {
            light = LED2_ON_BRIGHTNESS;
        } else {
            double abs_err = fabs(led2_err);
            if (abs_err > LED2_MAX_ERROR) abs_err = LED2_MAX_ERROR; // cap error
            float t = (float)(LED2_MAX_BRIGHTNESS / (LED2_MAX_ERROR - LED2_DEAD_BAND)); // dimness multiplier
            light = (uint8_t)(t * (LED2_MAX_ERROR - abs_err));
        }
        analogWrite(HEADING_LED_PIN, light);
    }

#endif

}



