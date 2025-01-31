#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

#include "Wire.h"

#endif

#include <Servo.h>


MPU6050 mpu;

 

Servo servo0;

Servo servo1;

Servo servo2;

float correct;

int j = 0;

 

#define OUTPUT_READABLE_YAWPITCHROLL

 

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

 

bool blinkState = false;

 

// MPU control/status vars

bool dmpReady = false;  // set true if DMP init was successful

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

uint16_t fifoCount;     // count of all bytes currently in FIFO

uint8_t fifoBuffer[64]; // FIFO storage buffer

 

// orientation/motion vars

Quaternion q;           // [w, x, y, z]         quaternion container

VectorInt16 aa;         // [x, y, z]            accel sensor measurements

VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements

VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

VectorFloat gravity;    // [x, y, z]            gravity vector

float euler[3];         // [psi, theta, phi]    Euler angle container

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

 

// packet structure for InvenSense teapot demo

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {

  mpuInterrupt = true;

}


 

void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

  Wire.begin();

  Wire.setClock(400000);

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE

  Fastwire::setup(400, true);

#endif


  Serial.begin(38400);

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

 

  // initialize device

  //Serial.println(F("Initializing I2C devices..."));

  mpu.initialize();

  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity

  mpu.setXGyroOffset(17);

  mpu.setYGyroOffset(-69);

  mpu.setZGyroOffset(27);

  mpu.setZAccelOffset(1551); // 1688 factory default for my test chip

 

  // make sure it worked (returns 0 if so)

  if (devStatus == 0) {

    // turn on the DMP, now that it's ready

    // Serial.println(F("Enabling DMP..."));

    mpu.setDMPEnabled(true);

 

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

 

    // get expected DMP packet size for later comparison

    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {

    // ERROR!

    // 1 = initial memory load failed

    // 2 = DMP configuration updates failed

    // (if it's going to break, usually the code will be 1)

    // Serial.print(F("DMP Initialization failed (code "));

    //Serial.print(devStatus);

    //Serial.println(F(")"));

  }

 

  // Define the pins to which the 3 servo motors are connected

  servo0.attach(10);

  servo1.attach(9);

  servo2.attach(8);

}


void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {

    if (mpuInterrupt && fifoCount < packetSize) {

      fifoCount = mpu.getFIFOCount();

    }

  }

 

  // reset interrupt flag and get INT_STATUS byte

  mpuInterrupt = false;

  mpuIntStatus = mpu.getIntStatus();

 

  // get current FIFO count

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {

    mpu.resetFIFO();

    fifoCount = mpu.getFIFOCount();

    Serial.println(F("FIFO overflow!"));

  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {


    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

 
    mpu.getFIFOBytes(fifoBuffer, packetSize);


    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL

    mpu.dmpGetQuaternion(&q, fifoBuffer);

    mpu.dmpGetGravity(&gravity, &q);

    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

 

    // Yaw, Pitch, Roll values - Radians to degrees

    ypr[0] = ypr[0] * 180 / M_PI;

    ypr[1] = ypr[1] * 180 / M_PI;

    ypr[2] = ypr[2] * 180 / M_PI;

    
    if (j <= 300) {

      correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings

      j++;

    }


    else {

      ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees


      int servo0Value = map(ypr[0], -90, 90, 0, 180);

      int servo1Value = map(ypr[1], -90, 90, 0, 180);

      int servo2Value = map(ypr[2], -90, 90, 180, 0);

      
      servo0.write(servo0Value);

      servo1.write(servo1Value);

      servo2.write(servo2Value);

    }

#endif

  }

}
