// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
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
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro; // <-- use for AD0 high
//MPU6050 accelgyro(MPU6050_ADDRESS_AD0_HIGH);

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t axb, ayb, azb;
int16_t gxb, gyb, gzb;

const double gyroLsb[] = {131.0, 65.5, 32.8, 16.4};
const double accelLsb[] = {16384.0, 8192.0, 4096.0, 2048.0};

const uint8_t gyroRate = MPU6050_GYRO_FS_2000;
const uint8_t accelRate = MPU6050_ACCEL_FS_16;

const uint16_t bias_count = 1000;

const double ST_MAX = 0.14;
const double ST_MIN = -0.14;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

void getBias() {
    int32_t sumAx = 0;
    int32_t sumAy = 0;
    int32_t sumAz = 0;
    int32_t sumGx = 0;
    int32_t sumGy = 0;
    int32_t sumGz = 0;
    for(uint32_t i = 0; i < bias_count; i++)
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumAx += ax;
        sumAy += ay;
        sumAz += az;

        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
    }

    gxb = sumGx/bias_count;
    gyb = sumGy/bias_count;
    gzb = sumGz/bias_count;

    axb = sumAx/bias_count;
    ayb = sumAy/bias_count;
    azb = sumAz/bias_count;
}

void setSelfTestBits(bool enabled) {
    accelgyro.setAccelXSelfTest(enabled);
    accelgyro.setAccelYSelfTest(enabled);
    accelgyro.setAccelZSelfTest(enabled);

    //accelgyro.setGyroXSelfTest(enabled);
    //accelgyro.setGyroYSelfTest(enabled);
    //accelgyro.setGyroZSelfTest(enabled);
}

// Return the factory trim value for accelerations
double getAccelFactoryTrim(uint8_t ftVal) {
    if(ftVal != 0) {
        return 4069*0.34*pow(0.92/0.34, (ftVal-1)/(pow(2,5)-2));
    } else {
        return 0.0;
    }
}

// Return the factory trim value for gyroscope
double getGyroFactoryTrim(uint8_t ftVal, bool isYAxis) {
    if(ftVal != 0) {
        double sign = isYAxis ? -1.0 : 1.0;
        return sign*25*131*pow(1.046,ftVal-1);
    } else {
        return 0.0;
    }
}

// Return the Change from Factory Trim of the Self-Test Response(%)
double getBITValue(double str, double ft) {
    return (str - ft)/ft;
}

// Execute a self test
bool selfTest() {
    bool ret = true;
    int16_t ax_st, ay_st, az_st;
    int16_t gx_st, gy_st, gz_st;

    // Save off current range settings
    int16_t gyroVal = accelgyro.getFullScaleGyroRange();
    int16_t accelVal = accelgyro.getFullScaleAccelRange();

    // Set to proper values for BIT
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    // Capture BIT and non-BIT values
    setSelfTestBits(true);
    delay(500); // Needed to get good values
    accelgyro.getMotion6(&ax_st, &ay_st, &az_st, &gx_st, &gy_st, &gz_st);
    setSelfTestBits(false);
    delay(500); // Needed to get good values
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax_st -= ax;
    ay_st -= ay;
    az_st -= az;

    gx_st -= gx;
    gy_st -= gy;
    gz_st -= gz;

    // Get the acceleration FT values
    double ax_ft = getAccelFactoryTrim(accelgyro.getAccelXSelfTestFactoryTrim());
    double ay_ft = getAccelFactoryTrim(accelgyro.getAccelYSelfTestFactoryTrim());
    double az_ft = getAccelFactoryTrim(accelgyro.getAccelZSelfTestFactoryTrim());

    // Get the gyroscope FT values
    double gx_ft = getGyroFactoryTrim(accelgyro.getGyroXSelfTestFactoryTrim(), false);
    double gy_ft = getGyroFactoryTrim(accelgyro.getGyroYSelfTestFactoryTrim(), true);
    double gz_ft = getGyroFactoryTrim(accelgyro.getGyroZSelfTestFactoryTrim(), false);

    // Calculate bit values
    double ax_bit = getBITValue(ax_st, ax_ft);
    double ay_bit = getBITValue(ay_st, ay_ft);
    double az_bit = getBITValue(az_st, az_ft);

    // Calculate bit values
    double gx_bit = getBITValue(gx_st, gx_ft);
    double gy_bit = getBITValue(gy_st, gy_ft);
    double gz_bit = getBITValue(gz_st, gz_ft);

    if(ax_bit < ST_MIN || ax_bit > ST_MAX) {
        Serial.println("AX Failed BIT");
        Serial.print("AX Bit Val: ");
        Serial.println(ax_bit);
        Serial.print("AX FT Val: ");
        Serial.println(ax_ft);
        Serial.print("AX STR Val: ");
        Serial.println(ax_st);
        ret = false;
    }
    if (ay_bit < ST_MIN || ay_bit > ST_MAX) {
        Serial.println("AY Failed BIT");
        Serial.print("AY Bit Val: ");
        Serial.println(ay_bit);
        Serial.print("AY FT Val: ");
        Serial.println(ay_ft);
        Serial.print("AY STR Val: ");
        Serial.println(ay_st);
        ret = false;
    }
    if (az_bit < ST_MIN || az_bit > ST_MAX) {
        Serial.println("AZ Failed BIT");
        Serial.print("AZ Bit Val: ");
        Serial.println(az_bit);
        Serial.print("AZ FT Val: ");
        Serial.println(az_ft);
        Serial.print("AZ STR Val: ");
        Serial.println(az_st);
        ret = false;
    }
    if (gx_bit < ST_MIN || gx_bit > ST_MAX) {
        Serial.println("GX Failed BIT");
        Serial.print("GX Bit Val: ");
        Serial.println(gx_bit);
        Serial.print("GX FT Val: ");
        Serial.println(gx_ft);
        Serial.print("GX STR Val: ");
        Serial.println(gx_st);
        ret = false;
    }
    if (gy_bit < ST_MIN || gy_bit > ST_MAX) {
        Serial.println("GY Failed BIT");
        Serial.print("GY Bit Val: ");
        Serial.println(gy_bit);
        Serial.print("GY FT Val: ");
        Serial.println(gy_ft);
        Serial.print("GY STR Val: ");
        Serial.println(gy_st);
        ret = false;
    }
    if (gz_bit < ST_MIN || gz_bit > ST_MAX) {
        Serial.println("GZ Failed BIT");
        Serial.print("GZ Bit Val: ");
        Serial.println(gz_bit);
        Serial.print("GZ FT Val: ");
        Serial.println(gz_ft);
        Serial.print("GZ STR Val: ");
        Serial.println(gz_st);
        ret = false;
    }

    // Reset values to original state
    accelgyro.setFullScaleGyroRange(gyroVal);
    accelgyro.setFullScaleGyroRange(accelVal);
    return ret;
}

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(1000000);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    accelgyro.setFullScaleGyroRange(gyroRate);
    accelgyro.setFullScaleAccelRange(accelRate);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values


    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
    if(!selfTest()) {
        exit(1);
    }
    getBias();
    accelgyro.setXGyroOffset(121);
    accelgyro.setYGyroOffset(55);
    accelgyro.setZGyroOffset(54);
}

void loop() {
    Serial.print("a/g:\t");
    Serial.print("ax(g):\t");
    Serial.print("ay(g):\t");
    Serial.print("az(g):\t");
    Serial.print("gx:\t");
    Serial.print("gy:\t");
    Serial.println("gz:\t");
    for(int i = 0; i < 1000; i++)
    {
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // these methods (and a few others) are also available
        //accelgyro.getAcceleration(&ax, &ay, &az);
        //accelgyro.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax/accelLsb[accelRate]); Serial.print("\t");
        Serial.print(ay/accelLsb[accelRate]); Serial.print("\t");
        Serial.print(az/accelLsb[accelRate]); Serial.print("\t");
        Serial.print((gx)/gyroLsb[gyroRate]); Serial.print("\t");
        Serial.print((gy)/gyroLsb[gyroRate]); Serial.print("\t");
        Serial.println((gz)/gyroLsb[gyroRate]);
        //Serial.print((gx-gxb)/gyroLsb[gyroRate]); Serial.print("\t");
        //Serial.print((gy-gyb)/gyroLsb[gyroRate]); Serial.print("\t");
        //Serial.println((gz-gzb)/gyroLsb[gyroRate]);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
#endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    exit(0);
}
