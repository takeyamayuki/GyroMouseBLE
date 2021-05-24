/* ===================== TO DO =====================
* - Add remote control of esp.reset()?
* - Add another sensor of some kind?
* 
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ================== USER CONFIG ==================

// Module ID
#define MODULE_ID "AFM_2"

// Networking setup
//#define COMPUTER_WIRED
#define COMPUTER_WIFI
//#define PUPSNET

// Which sensors used?
#define USE_MPU


// ==================================================

// WiFi network name and password:
#ifdef PUPSNET
const char * networkName = "PUPSnet";
const char * networkPswd = "apassword";
#else
const char * networkName = "stoatNet";
const char * networkPswd = "apassword";
#endif

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
//const char * udpAddress = "10.34.42.255";
//const char * udpAddress = "10.34.42.184";

#ifdef PUPSNET
const char * udpAddress = "10.34.42.18"; //Cuttlefish on pupsnet
#else
#ifdef COMPUTER_WIRED
const char * udpAddress = "192.168.1.2"; //Cuttlefish on stoatNet: 2 if wired
#endif
#ifdef COMPUTER_WIFI
const char * udpAddress = "192.168.1.3"; //Cuttlefish on stoatNet: 3 if on WiFi
#endif
#endif

const int udpPort = 3333;

// Are we currently connected to WiFi?
boolean connected = false;

int ledPin = 5;

//The udp library class
WiFiUDP udp;
//WiFiServer tcpServer(3335);

#define __PGMSPACE_H_ 1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino

// ==================================================

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

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

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


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

#ifdef OUTPUT_TEAPOT
// packet structure for InvenSense teapot demo
volatile uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
#endif

// ==================================================

static int i2cCore = 1;

#define INTERRUPT_PIN_MPU 19
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

volatile bool mpuDataReady = false;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

volatile int mpuDataCounter = 0;
int mpuDataCounterPrev = 0;

bool blinkState = false;

// ================================================================
// ===                INTERRUPT SERVICE ROUTINES                ===
// ================================================================

void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // initialize serial communication

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);

  // configure LED for output
  pinMode(ledPin, OUTPUT);

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);

  delay(100);

  Serial.print("Creating i2c task on core ");
  Serial.println(i2cCore);
  xTaskCreatePinnedToCore(
    sensorTask,   /* Function to implement the task */
    "coreTask", /* Name of the task */
    10000,      /* Stack size in words */
    NULL,       /* Task input parameter */
    20,          /* Priority of the task */
    NULL,       /* Task handle. */
    i2cCore);  /* Core where the task should run */

  Serial.println("i2c task created.");
}


// ================================================================
// ===              I2C SENSOR READ TASK (CORE 0)               ===
// ================================================================

void sensorTask( void * pvParameters ) {

  // ================== SETUP ==================

  delay(100);

  String taskMessage = "sensorTask running on core ";
  taskMessage = taskMessage + xPortGetCoreID();

  Serial.println(taskMessage);

  // join I2C bus (I2Cdev library doesn't do this automatically)

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  //      Wire.setClock(100000); // 100kHz I2C clock. Comment this line if having compilation difficulties

  delay(1000);

#ifdef USE_MPU

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  delay(100);

  mpu.reset(); //help startup reliably - doesn't always work though.
  // maybe can also reset i2c on esp32?

  delay(100);

  mpu.resetI2CMaster();

  delay(100);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN_MPU, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //    while (Serial.available() && Serial.read()); // empty buffer
  //    while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for someone else's test chip
  //  mpu.setZAccelOffset(0); // 1688 factory default for someone else's test chip


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MPU), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  delay(100);

#endif // USE_MPU

  // ================== LOOP ==================

  while (true) {
    if (connected) {

#ifdef USE_MPU

      // if programming failed, don't try to do anything
      if (!dmpReady) {
        Serial.println("dmpNotReady");
        delay(100);
      }

      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        //
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
      }

      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // full -180 to 180 deg Pitch, but other od behavior
        // mpu.dmpGetYawPitchRollBeng27(ypr, &q, &gravity);
        
        // for different board orientation. But Yaw doesn't work properly with this.
        // mpu.dmpGetYawPitchRollVertical(ypr, &q, &gravity);
        
        //        Serial.print("ypr\t");
        //        Serial.print(ypr[0] * 180 / M_PI);
        //        Serial.print("\t");
        //        Serial.print(ypr[1] * 180 / M_PI);
        //        Serial.print("\t");
        //        Serial.println(ypr[2] * 180 / M_PI);
        //                reportYpr = true;

#endif

#ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
//        mpu.dmpGetQuaternion(&q, fifoBuffer); // already got above
        mpu.dmpGetAccel(&aa, fifoBuffer);
//        mpu.dmpGetGravity(&gravity, &q); // already got above
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        //                Serial.print("areal\t");
        //                Serial.print(aaReal.x);
        //                Serial.print("\t");
        //                Serial.print(aaReal.y);
        //                Serial.print("\t");
        //                Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
//        mpu.dmpGetQuaternion(&q, fifoBuffer); // already got above
//        mpu.dmpGetAccel(&aa, fifoBuffer); // already got above
//        mpu.dmpGetGravity(&gravity, &q); // already got above
//        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // already got above
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        //                Serial.print("aworld\t");
        //                Serial.print(aaWorld.x);
        //                Serial.print("\t");
        //                Serial.print(aaWorld.y);
        //                Serial.print("\t");
        //                Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

        mpuDataCounter++;

        // blink LED to indicate activity - need to move this elsewhere
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
      }

#endif // USE_MPU

    } else { // not connected
      vTaskDelay(10); // wait and feed the watchdog timer
    }
  } // end of loop
} // end sensorTask



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (connected) {
    // UDP

    // improve to only xmit when changed
    // either compare to last, or keep track of when new data has been read
    // when new data increment a global counter. check here if different from last read
    // if so, send packet and update previousCounter val.
    // use separate counters for each sensor.

#ifdef USE_MPU
    if (mpuDataCounter != mpuDataCounterPrev) {
      // format: MODULE_ID mpu yaw pitch roll wx wy wz rx ry rz gravx gravy gravz
      udp.beginPacket(udpAddress, udpPort);
      udp.print(MODULE_ID);
      udp.print(" mpu ");
      //      udp.print(" ypr ");
      udp.print(ypr[0] * 180 / M_PI);
      udp.print(" ");
      udp.print(ypr[1] * 180 / M_PI);
      udp.print(" ");
      udp.print(ypr[2] * 180 / M_PI);
      udp.print(" ");

      //      udp.print(" aworld ");
      udp.print(aaWorld.x);
      udp.print(" ");
      udp.print(aaWorld.y);
      udp.print(" ");
      udp.print(aaWorld.z);
      udp.print(" ");

      //      udp.print(" areal ");
      udp.print(aaReal.x);
      udp.print(" ");
      udp.print(aaReal.y);
      udp.print(" ");
      udp.println(aaReal.z);

      udp.print(" ");
      udp.print(gravity.x);
      udp.print(" ");
      udp.print(gravity.y);
      udp.print(" ");
      udp.print(gravity.z);

      udp.endPacket();

      mpuDataCounterPrev = mpuDataCounter;
    }

#endif // USE_MPU

  } else { // not connected
    vTaskDelay(10); // wait and feed the watchdog timer.
  }
}

void connectToWiFi(const char * ssid, const char * pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      // When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      // initializes the UDP state
      // This initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpPort);
      //      tcpServer.begin();

      Serial.printf("Wifi Event running on core %d", (int)xPortGetCoreID());
      Serial.println();

      connected = true;

      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
  }
}
