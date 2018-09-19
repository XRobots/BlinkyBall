// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

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
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Adafruit Dotstar Lib ****************

#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 104 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    6
#define CLOCKPIN   7
Adafruit_DotStar strip = Adafruit_DotStar(
  NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);
  
// Adafruit Dotstar Lib ****************

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
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



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
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

double roll;
double pitch;
double yaw;

int roll2;
int pitch2;
int roll3;
int pitch3;

int but1;
int but2;
int mode = 0;
int modeFlag = 0;

int      head0  = 0, tail0 = -8; // Index of first 'on' and 'off' pixels
uint32_t color0 = 0xFF0000;      // 'On' color (starts red)

int      head1  = 0, tail1 = -8; // Index of first 'on' and 'off' pixels
uint32_t color1 = 0xFF0000;      // 'On' color (starts red)

int      head2  = 0, tail2 = -8; // Index of first 'on' and 'off' pixels
uint32_t color2 = 0xFF0000;      // 'On' color (starts red)

int      head3  = 0, tail3 = -8; // Index of first 'on' and 'off' pixels
uint32_t color3 = 0xFF0000;      // 'On' color (starts red)

int      head4  = 0, tail4 = -8; // Index of first 'on' and 'off' pixels
uint32_t color4 = 0xFF0000;      // 'On' color (starts red)

int      head5  = 0, tail5 = -8; // Index of first 'on' and 'off' pixels
uint32_t color5 = 0xFF0000;      // 'On' color (starts red)

int      head6  = 0, tail6 = -8; // Index of first 'on' and 'off' pixels
uint32_t color6 = 0xFF0000;      // 'On' color (starts red)

int      head7  = 0, tail7 = -8; // Index of first 'on' and 'off' pixels
uint32_t color7 = 0xFF0000;      // 'On' color (starts red)

int      head8  = 0, tail8 = -8; // Index of first 'on' and 'off' pixels
uint32_t color8 = 0xFF0000;      // 'On' color (starts red)

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(57600);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(10);
    mpu.setYGyroOffset(10);
    mpu.setZGyroOffset(53);
    mpu.setXAccelOffset(-264);
    mpu.setYAccelOffset(3441);
    mpu.setZAccelOffset(1491);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
      clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
    #endif
  
    strip.begin(); // Initialize pins for output
    strip.show();  // Turn all LEDs off ASAP
    
     // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    pinMode(47, INPUT_PULLUP);
    pinMode(49, INPUT_PULLUP);

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
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


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            
            pitch = ypr[1]* 180/M_PI;
            roll = ypr[2]* 180/M_PI;
            yaw = ypr[0]* 180/M_PI;

            roll2 = (roll+90)/13;
            pitch2 = (pitch+90)/13;

            roll3 = ((roll-90)/13)*-1;
            pitch3 = ((pitch-90)/13)*-1;            

            Serial.print(" twos ");
            Serial.print(pitch2);
            Serial.print(" , ");
            Serial.print(roll2);
            Serial.print(" threes ");
            Serial.print(pitch3);
            Serial.print(" , ");
            Serial.print(roll3);
            
        #endif

        //********** sort out mode select ***************

        but1 = digitalRead(47);
        but2 = digitalRead(49);

        if (but2 == 0 && modeFlag == 0) {             // menu select handling & debounce
          modeFlag = 1;
          mode = mode+1;
          mode = constrain(mode,0,4);
        }            
        else if (but1 == 0 && modeFlag == 0) {
          modeFlag = 1;
          mode = mode-1;
          mode = constrain(mode,0,4);
        }
        else if (but1 == 1 && but2 == 1){
          modeFlag = 0;
        }

        Serial.print(" modes ");
        Serial.print(but1);
        Serial.print(" , ");
        Serial.print(but2);
        Serial.print(" , ");
        Serial.println(mode);

        //************** do basic strip test / mode 0 **********************

        if (mode == 0) {          
            strip.setPixelColor(head0, color0); // 'On' pixel at head
            strip.setPixelColor(tail0, 0);     // 'Off' pixel at tail
            strip.show();                     // Refresh strip
            //delay(20);                        // Pause 20 milliseconds (~50 FPS)
          
            if(++head0 >= NUMPIXELS) {         // Increment head index.  Off end of strip?
              head0 = 0;                       //  Yes, reset head index to start
              if((color0 >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
                color0 = 0xFF0000;             //   Yes, reset to red
            }
            if(++tail0 >= NUMPIXELS) tail0 = 0; // Increment, reset tail index
        }    

        //************** do basic strip test 1 / mode 1 **********************

        if (mode == 1) {
            strip1(mode);
            strip2(mode);
            strip3(mode);
            strip4(mode);
            strip5(mode);
            strip6(mode);
            strip7(mode);
            strip8(mode);
        }

        else if (mode == 2) {

            //********* positive yaw ******
            if (yaw <10 && yaw > -10) {
              setall(0,104,0x000000);     // turn all off
            }          
            else if (yaw > 10 && yaw < 30){
                setall(0,13,0x0000FF);    
                setall(13,26,0x000000);      
            }
            else if (yaw > 30 && yaw < 40) { 
                setall(13,26,0x0000FF); 
                setall(26,39,0x000000);
            }
            else if (yaw > 40 && yaw < 60) { 
                setall(26,39,0x0000FF); 
                setall(39,52,0x000000); 
            }
            else if (yaw > 60 && yaw < 70) { 
                setall(39,52,0x0000FF);
                setall(52,65,0x000000);  
            }
            else if (yaw > 70 && yaw < 80) { 
                setall(52,65,0x0000FF);
                setall(65,78,0x000000);
            } 
            else if (yaw > 80 && yaw < 90) { 
                setall(65,78,0x0000FF);
                setall(78,91,0x000000);
            } 
            else if (yaw > 90 && yaw < 100) { 
                setall(78,91,0x0000FF);
                setall(91,104,0x000000);
            } 
            else if (yaw > 100) { 
                setall(91,104,0x0000FF);
            } 
            
            //********** negative yaw ******
            
          
            else if (yaw < -10 && yaw > -30){
                setall(91,104,0xFF0000);    
                setall(78,91,0x000000);      
            }
            else if (yaw < -30 && yaw > -40) { 
                setall(78,91,0xFF0000); 
                setall(65,78,0x000000);
            }
            else if (yaw < -40 && yaw > -60) { 
                setall(65,78,0xFF0000); 
                setall(52,65,0x000000); 
            }
            else if (yaw < -60 && yaw > -70) { 
                setall(52,65,0xFF0000);
                setall(39,52,0x000000);  
            }
            else if (yaw < -70 && yaw > -80) { 
                setall(39,52,0xFF0000);
                setall(26,39,0x000000);
            } 
            else if (yaw < -80 && yaw > -90) { 
                setall(26,39,0xFF0000);
                setall(13,26,0x000000);
            } 
            else if (yaw < -90 && yaw > -100) { 
                setall(13,26,0xFF0000);
                setall(0,13,0x000000);
            } 
            else if (yaw < -100) { 
                setall(0,13,0xFF0000);
            }
            
            strip.show();             
        }

        if (mode == 3) {
            setall(0,104, 0x000000);    // set all off before slecting which should be on
           
            if (pitch > 5 && roll > 5) {
              setall(26,39, 0x00FF00);
            }
            else if (pitch < -5 && roll < -5) {
              setall(78,91, 0x00FF00);
            }
            else if (pitch > 5 && roll < -5){
              setall(0,13, 0x00FF00);
            }
            else if (pitch < -5 && roll > 5) {
              setall(52,65, 0x00FF00);
            }
            else if (roll > 5) {
              setall(39,52, 0x00FF00);
            }
            else if (roll < -5) {
              setall(91,104, 0x00FF00);
            }
            else if (pitch < -5) {
              setall(65,78, 0x00FF00);
            }
            else if (pitch > 5){
              setall(13,26, 0x00FF00);
            }
            strip.show();    
        }

        if (mode == 4) {
            setall(0,104, 0x000000);    // set all off before slecting which should be on

            setall((52-roll2), 52, 0xFF0000);
            setall((91+roll2), 104, 0xFF0000);   
            setall((26-pitch2), 26, 0xFF0000);     
            setall((65+pitch2), 78, 0xFF0000);

            setall((39-((roll2+pitch2)/2)), 39, 0xFF0000);
            setall((78+((roll2+pitch2)/2)), 91, 0xFF0000);

            setall((13-((roll3+pitch2)/2)), 13, 0xFF0000);
            setall((52+((roll3+pitch2)/2)), 65, 0xFF0000);
            
            strip.show(); 
        }

          
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

//******* set all LEDs ***********

void setall(int start, int finish, uint32_t color) {
      for (int i = start; i < finish; i = i + 1){
        strip.setPixelColor(i, color);
        }
}

void strip1(int mode) {

    if (mode == 1) {
        
    strip.setPixelColor(head1, color1); // 'On' pixel at head
    strip.setPixelColor(tail1, 0);     // 'Off' pixel at tail
  
    if(++head1 >= 13) {         // Increment head index.  Off end of strip?
      head1 = 0;                       //  Yes, reset head index to start
      if((color1 >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
        color1 = 0xFF0000;             //   Yes, reset to red
    }
    if(++tail1 >= 13) tail1 = 0; // Increment, reset tail index
    } // end of mode 1

}

void strip2(int mode) {

    if (mode == 1) {
        
    strip.setPixelColor(head2, color2); // 'On' pixel at head
    strip.setPixelColor(tail2, 13);     // 'Off' pixel at tail
    strip.show();                     // Refresh strip
  
    if(++head2 >= 26) {         // Increment head index.  Off end of strip?
      head2 = 13;                       //  Yes, reset head index to start
      if((color2 >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
        color2 = 0xFF0000;             //   Yes, reset to red
    }
    if(++tail2 >= 26) tail2 = 13; // Increment, reset tail index
    } // end of mode 1
}


void strip3(int mode) {

    if (mode == 1) {
        
    strip.setPixelColor(head3, color3); // 'On' pixel at head
    strip.setPixelColor(tail3, 26);     // 'Off' pixel at tail
    strip.show();                     // Refresh strip
  
    if(++head3 >= 39) {         // Increment head index.  Off end of strip?
      head3 = 26;                       //  Yes, reset head index to start
      if((color3 >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
        color3 = 0xFF0000;             //   Yes, reset to red
    }
    if(++tail3 >= 39) tail3 = 26; // Increment, reset tail index
    } // end of mode 1

}

void strip4(int mode) {

    if (mode == 1) {
        
    strip.setPixelColor(head4, color4); // 'On' pixel at head
    strip.setPixelColor(tail4, 39);     // 'Off' pixel at tail
    strip.show();                     // Refresh strip
  
    if(++head4 >= 52) {         // Increment head index.  Off end of strip?
      head4 = 39;                       //  Yes, reset head index to start
      if((color4 >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
        color4 = 0xFF0000;             //   Yes, reset to red
    }
    if(++tail4 >= 52) tail4 = 39; // Increment, reset tail index
    } // end of mode 1
}

void strip5(int mode) {

    if (mode == 1) {
        
    strip.setPixelColor(head5, color5); // 'On' pixel at head
    strip.setPixelColor(tail5, 52);     // 'Off' pixel at tail
    strip.show();                     // Refresh strip
  
    if(++head5 >= 65) {         // Increment head index.  Off end of strip?
      head5 = 52;                       //  Yes, reset head index to start
      if((color5 >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
        color5 = 0xFF0000;             //   Yes, reset to red
    }
    if(++tail5 >= 65) tail5 = 52; // Increment, reset tail index
    } // end of mode 1
}

void strip6(int mode) {

    if (mode == 1) {
        
    strip.setPixelColor(head6, color6); // 'On' pixel at head
    strip.setPixelColor(tail6, 65);     // 'Off' pixel at tail
    strip.show();                     // Refresh strip
  
    if(++head6 >= 78) {         // Increment head index.  Off end of strip?
      head6 = 65;                       //  Yes, reset head index to start
      if((color6 >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
        color6 = 0xFF0000;             //   Yes, reset to red
    }
    if(++tail6 >= 78) tail6 = 65; // Increment, reset tail index
    } // end of mode 1
}

void strip7(int mode) {

    if (mode == 1) {
        
    strip.setPixelColor(head7, color7); // 'On' pixel at head
    strip.setPixelColor(tail7, 78);     // 'Off' pixel at tail
    strip.show();                     // Refresh strip
  
    if(++head7 >= 91) {         // Increment head index.  Off end of strip?
      head7 = 78;                       //  Yes, reset head index to start
      if((color7 >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
        color7 = 0xFF0000;             //   Yes, reset to red
    }
    if(++tail7 >= 91) tail7 = 78; // Increment, reset tail index
    } // end of mode 1
}

void strip8(int mode) {

    if (mode == 1) {
        
    strip.setPixelColor(head8, color8); // 'On' pixel at head
    strip.setPixelColor(tail8, 91);     // 'Off' pixel at tail
    strip.show();                     // Refresh strip
  
    if(++head8 >= 104) {         // Increment head index.  Off end of strip?
      head8 = 91;                       //  Yes, reset head index to start
      if((color8 >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
        color8 = 0xFF0000;             //   Yes, reset to red
    }
    if(++tail8 >= 104) tail8 = 91; // Increment, reset tail index
    } // end of mode 1
}


