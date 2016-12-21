* ============================================
  Author: Emerson Garland

  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

//I2C Libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//SPI Library
#include <SPI.h>

//ModbusTCP Library
#include "Mudbus.h"

//Ethernet Library *only works with wiznet 5100 chip
#include <Ethernet.h>

MPU6050 accelgyro;
MPU6050 mpu;

unsigned long previousMillis = 0;
const long interval = 30000;
int16_t previousYaw;
int16_t previousPitch;
int16_t previousRoll;
int16_t previousAccelX;
int16_t previousAccelY;
int16_t previousAccelZ;
int16_t previousHighYaw;
int16_t previousLowYaw;
int16_t previousHighPitch;
int16_t previousLowPitch;
int16_t previousHighRoll;
int16_t previousLowRoll;
int16_t currentLowYaw;
int16_t currentLowPitch;
int16_t currentLowRoll;
int16_t currentHighYaw;
int16_t currentHighPitch;
int16_t currentHighRoll;
int16_t currentAccelX;
int16_t currentAccelY;
int16_t currentAccelZ;
int16_t maxYaw;
int16_t maxPitch;
int16_t maxRoll;
int16_t minYaw;
int16_t minPitch;
int16_t minRoll;
int16_t maxAccelX;
int16_t maxAccelY;
int16_t maxAccelZ;
int16_t diffYaw;
int16_t diffPitch;
int16_t diffRoll;
int sameValue;
int oldYaw;


int16_t ax, ay, az;
int16_t gx, gy, gz;
uint8_t devStatus;
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
//#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL


#define LED_PIN 13
bool blinkState = false;

Mudbus Mb;
//Function codes 1(read coils), 3(read registers), 5(write coil), 6(write register)
//signed int Mb.R[0 to 125] and bool Mb.C[0 to 128] MB_N_R MB_N_C
//Port 502 (defined in Mudbus.h) MB_PORT



void setup() {

  uint8_t mac[]     = { 0x90, 0xA2, 0xDA, 0x00, 0x51, 0x06 };
  uint8_t ip[]      = { 192, 168, 100, 200 };
  //uint8_t gateway[] = { 192, 168, 100, 1 };
  uint8_t subnet[]  = { 255, 255, 255, 0 };
  Ethernet.begin(mac, ip, subnet);
  //Avoid pins 4,10,11,12,13 when using ethernet shield

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize device
  //Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  mpu.initialize();
  // verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // use the code below to change accel/gyro offset values
  mpu.setXGyroOffset(65);
  mpu.setYGyroOffset(22);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(2448);
  mpu.setYAccelOffset(-746);
  mpu.setZAccelOffset(1403); 
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

  }
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void(* resetFunc) (void) = 0;


void loop() {

  while (fifoCount < packetSize)
  {
    fifoCount = mpu.getFIFOCount();
  }
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.resetFIFO();
  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
  // display tab-separated accel/gyro x/y/z values
  //Serial.print("a/g:\t");
  /*          Serial.print(ax); Serial.print("\t");
              Serial.print(ay); Serial.print("\t");
              Serial.print(az); Serial.print("\t");
              Serial.print(gx); Serial.print("\t");
              Serial.print(gy); Serial.print("\t");
              Serial.println(gz);
  */
#endif

#ifdef OUTPUT_READABLE_QUATERNION
  // display quaternion values in easy matrix form: w x y z
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  /*           Serial.print("quat\t");
              Serial.print(q.w);
              Serial.print("\t");
              Serial.print(q.x);
              Serial.print("\t");
              Serial.print(q.y);
              Serial.print("\t");
              Serial.println(q.z);
  */
#endif

#ifdef OUTPUT_READABLE_EULER
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  /*          Serial.print("euler\t");
              Serial.print(euler[0] * 180/M_PI);
              Serial.print("\t");
              Serial.print(euler[1] * 180/M_PI);
              Serial.print("\t");
              Serial.println(euler[2] * 180/M_PI);
  */
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  /*           Serial.print("ypr\t");
              Serial.print(ypr[0] * 180/M_PI);
              Serial.print("\t");
              Serial.print(ypr[1] * 180/M_PI);
              Serial.print("\t");
              Serial.println(ypr[2] * 180/M_PI);
  */
#endif

#ifdef OUTPUT_READABLE_REALACCEL
  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  /*          Serial.print("areal\t");
              Serial.print(aaReal.x);
              Serial.print("\t");
              Serial.print(aaReal.y);
              Serial.print("\t");
              Serial.println(aaReal.z);
  */
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  /*          Serial.print("aworld\t");
              Serial.print(aaWorld.x);
              Serial.print("\t");
              Serial.print(aaWorld.y);
              Serial.print("\t");
              Serial.println(aaWorld.z);
  */
#endif

//#ifdef OUTPUT_BINARY_ACCELGYRO
//  Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
//  Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
//  Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
//  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
//  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
//  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
//#endif

  // blink LED to indicate activity
  //blinkState = !blinkState;
  //digitalWrite(LED_PIN, blinkState);


  Mb.Run();

  //ModbusTCP Register Mapping
  //Analog inputs 0-1023
  Mb.R[1] = (ypr[0] * 180 / M_PI * 100); //Yaw * 100 for scaling int for 2 decimal places
  Mb.R[2] = (ypr[1] * 180 / M_PI * 100); //Pitch * 100 for scaling int for 2 decimal places
  Mb.R[3] = (ypr[2] * 180 / M_PI * 100); //Roll * 100 for scaling int for 2 decimal places
  Mb.R[4] = (aaReal.x); //Acceleration X
  Mb.R[5] = (aaReal.y); //Acceleration Y
  Mb.R[6] = (aaReal.z); //Acceleration Z
  Mb.R[7] = (euler[0] * 180 / M_PI * 100); //Euler X
  Mb.R[8] = (euler[1] * 180 / M_PI * 100); //Euler Y
  Mb.R[9] = (euler[2] * 180 / M_PI * 100); //Euler Z
  Mb.R[10] = (aaWorld.x); //World Acceleration X
  Mb.R[11] = (aaWorld.y); //World Acceleration Y
  Mb.R[12] = (aaWorld.z); //World Acceleration Z
  Mb.R[13] = (previousHighYaw); //Yaw * 100 for scaling int for 2 decimal places
  Mb.R[14] = (previousHighPitch); //Yaw * 100 for scaling int for 2 decimal places
  Mb.R[15] = (previousHighRoll); //Yaw * 100 for scaling int for 2 decimal places
  Mb.R[16] = (maxYaw); //Max Yaw over configurable time interval
  Mb.R[17] = (maxPitch); //Max Pitch over configurable time interval
  Mb.R[18] = (maxRoll); //Max Roll over configurable time interval
  Mb.R[19] = (maxAccelX); //Max Acceleration along the X-Axis over configurable time interval
  Mb.R[20] = (maxAccelY); //Max Acceleration along the Y-Axis over configurable time interval
  Mb.R[21] = (maxAccelZ); //Max Acceleration along the Z-Axis over configurable time interval
  Mb.R[22] = (previousLowYaw); //Yaw * 100 for scaling int for 2 decimal places
  Mb.R[23] = (previousLowPitch); //Yaw * 100 for scaling int for 2 decimal places
  Mb.R[24] = (previousLowRoll); //Yaw * 100 for scaling int for 2 decimal places
  Mb.R[25] = abs(Mb.R[15] - Mb.R[24]);
  Mb.R[26] = (diffYaw);
  Mb.R[27] = (diffPitch);
  Mb.R[28] = (diffRoll);
  //*************************************************************************************


  currentLowYaw = Mb.R[1];
  currentLowPitch = Mb.R[2];
  currentLowRoll = Mb.R[3];
  currentHighYaw = Mb.R[1];
  currentHighPitch = Mb.R[2];
  currentHighRoll = Mb.R[3];
  currentAccelX = abs(Mb.R[4]);
  currentAccelY = abs(Mb.R[5]);
  currentAccelZ = abs(Mb.R[6]);
  //  currentYawDev = (Mb.R[1]);
  //  currentPitchDev = (Mb.R[2]);
  //  currentRollDev = (Mb.R[3]);

  //Declare currentMillis the millis() function; time from internal clock
  unsigned long currentMillis = millis();
  //if the current time - previous time is less than the constant interval
  //defined above and currentYaw is greater than the previousYaw, assign currentYaw's value to previousYaw
//Serial.println("im just before the if statement");
  if (currentMillis - previousMillis >= interval) {
      if (oldYaw == currentHighYaw) {
        Serial.println("greater than");
        sameValue = sameValue + 1;
        Serial.println(sameValue);
        
        //Serial.println(sameValue);
        //Serial.println("same value");
        delay(1000);
        //Serial.println(sameValue);
        //delay(1000);
      
      if (sameValue == 100) {
        //Serial.println("going down for reset");
        resetFunc();
      }
    }
  }
  if (currentMillis - previousMillis < interval) {
    if (currentHighYaw > previousHighYaw) {
      oldYaw = previousHighYaw;
      previousHighYaw = currentHighYaw;
    }
    if (currentLowYaw < previousLowYaw) {
      previousLowYaw = currentLowYaw;
    }
  }
  if (currentMillis - previousMillis < interval) {
    if (currentHighPitch > previousHighPitch) {
      previousHighPitch = currentHighPitch;
    }
    if (currentLowPitch < previousLowPitch) {
      previousLowPitch = currentLowPitch;
    }
  }
  if (currentMillis - previousMillis < interval) {
    if (currentHighRoll > previousHighRoll) {
      previousHighRoll = currentHighRoll;
    }
    if (currentLowRoll < previousLowRoll) {
      previousLowRoll = currentLowRoll;
    }
  }
  if (currentMillis - previousMillis < interval) {
    if (currentAccelX > previousAccelX) {
      previousAccelX = currentAccelX;
    }
  }
  if (currentMillis - previousMillis < interval) {
    if (currentAccelY > previousAccelY) {
      previousAccelY = currentAccelY;
    }
  }
  if (currentMillis - previousMillis < interval) {
    if (currentAccelZ > previousAccelZ) {
      previousAccelZ = currentAccelZ;
    }
  }

  //if current time - previous time is greater than or equal to the interval defined above,
  //make previous time equal to current time, set maxroll equal to the previous roll
  //(stored over from the above if then statements) and reset previous values

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    maxYaw = abs(previousHighYaw);
    maxPitch = abs(previousHighPitch);
    maxRoll = abs(previousHighRoll);

    minYaw = abs(previousLowYaw);
    minPitch = abs(previousLowPitch);
    minRoll = abs(previousLowRoll);

    maxAccelX = previousAccelX;
    maxAccelY = previousAccelY;
    maxAccelZ = previousAccelZ;

    previousHighYaw = -9999;
    previousHighPitch = -9999;
    previousHighRoll = -9999;

    previousLowYaw = 9999;
    previousLowPitch = 9999;
    previousLowRoll = 9999;

    previousAccelX = 0;
    previousAccelY = 0;
    previousAccelZ = 0;

    diffYaw = abs(maxYaw - minYaw);
    diffRoll = abs(maxRoll - minRoll);
    diffPitch = abs(maxPitch - minPitch);

  }


  delay(10);
  Serial.println(Mb.R[26]);
  //Serial.print(", ");
  //Serial.print(Mb.R[27]);
  //Serial.print(", ");
  //Serial.println(Mb.R[28]);

  //Serial.println("test");
  //Serial.print(", ");
  //Serial.print(Mb.R[4]);
  //Serial.print(", ");
  //Serial.print(Mb.R[5]);
  //Serial.print(", ");
  //Serial.println(Mb.R[6]);

  mpu.resetFIFO();
}
