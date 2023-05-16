
#include <Arduino.h>
#include <BleGamepad.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// TwoWire I2CMPU = TwoWire(0);

const int SCLpin = 26;
const int SDApin = 27;

MPU6050 mpu;

#define INTERRUPT_PIN 25

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

BleGamepad bleGamepad;

const int potPin = 34;                // Potentiometer is connected to GPIO 2 (Analog ADC1_CH0)
const int numberOfPotSamples = 5;     // Number of pot samples to take (to smooth the values)
const int delayBetweenSamples = 4;    // Delay in milliseconds between pot samples
const int delayBetweenHIDReports = 5; // Additional delay in milliseconds between HID reports

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDApin, SCLpin);
  Serial.println(F("Initializing I2C devices..."));

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));

  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));

  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
                             // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  Serial.println("Starting BLE work!");
  bleGamepad.begin();
}

void loop()
{

  if (bleGamepad.isConnected())
  {

    //  interrupts();

    /* Get new sensor events with the readings */
    // delay(1000);
    // read a packet from FIFO
    if (true)
    { // dmpReady ||
      bool mpur = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
      Serial.print(mpur);
      Serial.print(dmpReady);
      if (mpur)
      { // Get the Latest packet

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      }
    }

    // Serial.print("ypr\t");
    // Serial.print(ypr[0] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.println(ypr[2] * 180 / M_PI);

    int potValues[numberOfPotSamples]; // Array to store pot readings
    int potValue = analogRead(potPin);             // Variable to store calculated pot reading average


    // Calculate the average
    float yr =  ypr[0] * 180/M_PI;
    float pr =  ypr[1] * 180/M_PI;
    float rr =  ypr[2] * 180/M_PI;

    // Map analog reading from 0 ~ 4095 to 32737 ~ 0 for use as an axis reading
    int adjustedFlex = map(potValue, 0, 4095, 32737, 0);
    int adjustedY = map(yr, -180, 180, 32737, 0);
    int adjustedP = map(pr, -180, 180, 32737, 0);
    int adjustedR = map(rr, -180, 180, 32737, 0);
    // Update X axis and auto-send report
    // bleGamepad.setX(adjustedFlex);
    // bleGamepad.setLeftTrigger(adjustedFlex);
    // bleGamepad.setLeftThumb(adjustedY, adjustedP);
    bleGamepad.setAxes(adjustedFlex, adjustedY, adjustedP, adjustedR);
    delay(delayBetweenHIDReports);

    // The code below (apart from the 2 closing braces) is for pot value degugging, and can be removed
    // Print readings to serial port

    Serial.print("Sent: ");

    Serial.print(adjustedFlex);

    Serial.print("\tY: ");
    Serial.print(yr);
    Serial.print("\tP: ");
    Serial.print(pr);
    Serial.print("\tR: ");
    Serial.print(rr);

    Serial.print("\tRaw Avg: ");
    Serial.println(potValue);
    // Serial.print("\tRaw: {");

    // // Iterate through raw pot values, printing them to the serial port
    // for (int i = 0; i < numberOfPotSamples; i++)
    // {
    //     Serial.print(potValues[i]);

    //     // Format the values into a comma seperated list
    //     if (i == numberOfPotSamples - 1)
    //     {
    //         Serial.println("}");
    //     }
    //     else
    //     {
    //         Serial.print(", ");
    //     }
    // }
  }
}