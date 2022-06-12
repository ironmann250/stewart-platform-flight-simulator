#include <ServoEasing.hpp>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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

ServoEasing servo1,servo2,servo3,servo4,servo5,servo6;
String inputString;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  servo1.attach(12);
  servo2.attach(11);
  servo3.attach(10);
  servo4.attach(9);
  servo5.attach(8);
  servo6.attach(7);

  Serial.begin(115200);
  setSpeedForAllServos(300);
  // put your setup code here, to run once:
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial); 

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    Serial.println(F("Done init"));
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    Serial.println(F("offset"));
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      Serial.println(F("Dev 0"));
      // Calibration Time: generate offsets and calibrate our MPU6050
      Serial.println(F("cal accel"));
      mpu.CalibrateAccel(6);
      Serial.println(F("cal gyro"));
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      Serial.println(F("cone cal"));
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
      } else {
      Serial.println(F("dev error"));
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
  delay(500);
}

void loop() 
  {
    while(Serial.available())
  {
  
    char inChar = (char)Serial.read();
    inputString += inChar;
    if(inChar == '\n')
    {
      handle_incoming(inputString);
      //Serial.println(inputString);
      inputString = "";
    }
  }
  read_imu();
}

void handle_incoming(String inputString)
{
  char receivedChars[inputString.length()];
  for(int i=0;i<inputString.length();i++){
    receivedChars[i] = inputString[i];
  }
  
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(receivedChars,",");      // get the first part - the string
  float sservo1= atoi(strtokIndx); // copy it to messageFromPC
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  float sservo2 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  float sservo3 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  float sservo4 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  float sservo5 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, "\n"); // this continues where the previous call left off
  float sservo6 = atoi(strtokIndx); 
  //Serial.println(sservo1);
  //Serial.println(sservo2);
  //Serial.println(sservo3);
  //Serial.println(sservo4);
  //Serial.println(sservo5);
  //Serial.println(sservo6);

  //phi is the "tool angle" 
  goto_angle(sservo1,sservo2,sservo3,sservo4,sservo5,sservo6);
}

void goto_angle(float sservo1,float sservo2,float sservo3,float sservo4,float sservo5,float sservo6)
{
  setSpeedForAllServos(300);
    servo1.setEaseTo(180-sservo1);
    servo2.setEaseTo(sservo2);
    servo3.setEaseTo(180-sservo3);
    servo4.setEaseTo(sservo4);
    servo5.setEaseTo(180-sservo5);
    servo6.setEaseTo(sservo6);
    synchronizeAllServosStartAndWaitForAllServosToStop();
}

void read_imu()
{

  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr");
            
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("/");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("/");
            Serial.println(ypr[2] * 180/M_PI);
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
