#include <Wire.h>

#include "GNOR_V4.h"

/*---Servo library and objects (shared by USE_BOAT and USE_SERVO_TEST)---*/
#include <Servo.h>

Servo servo1;    // Rudder
Servo servo2;    // Left motor (dual motor config)
Servo servo3;    // Auxiliary servo
Servo servoEsc;  // Single motor OR right motor (dual motor config)


#ifdef USE_LED_TEST
/*---LED Test Variables---*/
unsigned long ledLastChange = 0;
#define LED_INTERVAL_MS 500  // ms per color
#endif                          // USE_LED_TEST

#ifdef USE_SERVO_TEST
/*---Servo Test Variables---*/
int servoPos = 0;
bool sweepUp = true;
#endif  // USE_SERVO_TEST

#ifdef USE_BOAT
void boatLoop(unsigned long timestamp, double heading);
#endif  // USE_BOAT

double yaw = 1.0;

#ifdef USE_MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gy;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

#endif  // USE_MPU

void setup() {
  Serial.begin(115200);
  while (!Serial && (millis() < 3000));
    ;  // wait for Serial startup

  Serial.println("System Starting");

  // Enable pull-up resistors on active LOW switches
  pinMode(MOTOR_SWITCH, INPUT_PULLUP);
  pinMode(CALIBRATE_SWITCH, INPUT_PULLUP);

#ifdef USE_EXT_PULLUPS
// Disable internal pull-up resistors on I2C lines
pinMode(A4, INPUT);
pinMode(A5, INPUT);
#endif

#ifdef USE_LEDS
pinMode(STARTUP_LED_PIN, OUTPUT);
pinMode(DRIFT_LED_PIN, OUTPUT);
pinMode(HEADING_LED_PIN, OUTPUT);
#endif

#ifdef USE_MPU
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  delay(200);  // Allow MPU6050 I2C bus to fully settle after power-on or reset
  mpu.initialize();

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initialize and configure the DMP — retry up to 3 times */
  Serial.println(F("Initializing DMP..."));
  int dmpTries = 0;
  do {
    if (dmpTries > 0) {
      Serial.print(F("DMP init failed (code "));
      Serial.print(devStatus);
      Serial.print(F("), retrying (attempt "));
      Serial.print(dmpTries + 1);
      Serial.println(F(")..."));
      delay(200);
    }
    devStatus = mpu.dmpInitialize();
  } while (devStatus != 0 && ++dmpTries < 3);

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));  //Turning ON DMP
    mpu.setDMPEnabled(true);

    /* Disable MPU6050 hardware interrupt at the chip level (INT_ENABLE = 0x00),
       then read INT_STATUS to clear any pending interrupt */
    mpu.setIntEnabled(0);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Polling mode (no interrupt)..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();  //Get expected DMP packet size for later comparison
  } else {
    Serial.print(F("DMP Initialization failed (code "));  //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
#endif  // USE_MPU

#ifdef USE_LEDS
  Serial.println(F("LEDs enabled."));
#endif  // USE_LEDS


  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servoEsc.attach(ESC_PIN);
  servo1.write(90);

#ifdef USE_SERVO_TEST
  Serial.println(F("Servo test enabled."));
#endif  // USE_SERVO_TEST

#ifdef USE_EXT_PULLUPS
// Reset pins to normal inputs after initializing I2C
pinMode(A4, INPUT);
pinMode(A5, INPUT);
#endif
}

//
// MAIN LOOP
//
void loop() {
#ifdef USE_MPU
  if (!DMPReady) return;  // Stop the program if DMP initialization failed.

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Get the Latest packet
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180 / M_PI;
    unsigned long timestamp = millis();

    //Serial.print("yaw: ");
    //Serial.println(yaw);

#ifdef USE_BOAT
    boatLoop(timestamp, yaw);
#endif  // USE_BOAT
  }
#else 
  boatLoop(millis(), 0.0); // open loop: disregard yaw
#endif  // USE_MPU

#ifdef USE_LED_TEST
  /* Cycle all three LEDs in steps 
  LED 1: startup LED
  LED 2: drift LED
  LED 3: heading LED */
  unsigned long now = millis();
  bool ledsLit[3] = {false, false, false};
  int numLeds = sizeof(ledsLit) / sizeof(ledsLit[0]);
  int ledIndex = 0;

  if (now - ledLastChange >= LED_INTERVAL_MS) {
    ledLastChange = now;
    ledsLit[ledIndex] = !ledsLit[ledIndex];
    if(++ledIndex >= numLeds) ledIndex = 0;

    /* Test switches while we are here 
    For each combo, turn an LED solid*/
    bool motorPressed = (digitalRead(MOTOR_SWITCH) == LOW);
    bool calibratePressed = (digitalRead(CALIBRATE_SWITCH) == LOW);
    if (motorPressed)
      ledsLit[0]=true; ledsLit[1]=false; ledsLit[2]=false;
    else if (calibratePressed)
      ledsLit[0]=false; ledsLit[1]=true; ledsLit[2]=false;
    else if (motorPressed && calibratePressed)
      ledsLit[0]=false; ledsLit[1]=false; ledsLit[2]=true;
  }
  digitalWrite(STARTUP_LED_PIN, ledsLit[0]);
  digitalWrite(DRIFT_LED_PIN, ledsLit[1]);
  digitalWrite(HEADING_LED_PIN, ledsLit[2]);
#endif  // USE_LED_TEST

#ifdef USE_SERVO_TEST
  /* Sweep all servos from low (20°) to high (160°) and back */
  servo1.write(servoPos);
  servo2.write(servoPos);
  servo3.write(servoPos);
  servoEsc.write(servoPos);

  Serial.print(F("Servo pos: "));
  Serial.println(servoPos);

  if (sweepUp) {
    servoPos++;
    if (servoPos >= 160) sweepUp = false;
  } else {
    servoPos--;
    if (servoPos <= 20) sweepUp = true;
  }
  delay(15);
#endif  // USE_SERVO_TEST
}
