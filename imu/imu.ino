// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// SD library for reading/writing to SD card
#include <SD.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro; // <-- use for AD0 high
//MPU6050 accelgyro(MPU6050_ADDRESS_AD0_HIGH);

File datafile;
const char* filename = "imu.out";

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

// Allow outputs to both the card and the serial port for now
#define __BOTH_OUTPUTS__

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

// These templated functions are here to make it easy to
// print things to either the serial interface, card interface, or
// both.
template<typename T> void println(T message) {
#if defined (__CARD_ONLY__) || defined (__BOTH_OUTPUTS__)
    if(datafile) {
        datafile.println(message);
    }
#endif
#if defined (__SERIAL_ONLY__) || defined (__BOTH_OUTPUTS__)
    if(Serial) {
        Serial.println(message);
    }
#endif
}

template<typename T> void print(T message) {
#if defined (__CARD_ONLY__) || defined (__BOTH_OUTPUTS__)
    if(datafile) {
        datafile.print(message);
    }
#endif
#if defined (__SERIAL_ONLY__) || defined (__BOTH_OUTPUTS__)
    if(Serial) {
        Serial.print(message);
    }
#endif
}

template<typename T> size_t write(T val) {
#if defined (__CARD_ONLY__) || defined (__BOTH_OUTPUTS__)
    if(datafile) {
        return datafile.write(val);
    }
#endif
#if defined (__SERIAL_ONLY__) || defined (__BOTH_OUTPUTS__)
    if(Serial) {
        return Serial.write(val);
    }
#endif
    return 0;
}

template<typename T> size_t write(T* buf, size_t len) {
#if defined (__CARD_ONLY__) || defined (__BOTH_OUTPUTS__)
    if(datafile) {
        return datafile.write(buf, len);
    }
#endif
#if defined (__SERIAL_ONLY__) || defined (__BOTH_OUTPUTS__)
    if(Serial) {
        return Serial.write(buf, len);
    }
#endif
    return 0;
}



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
        println("AX Failed BIT");
        print("AX Bit Val: ");
        println(ax_bit);
        print("AX FT Val: ");
        println(ax_ft);
        print("AX STR Val: ");
        println(ax_st);
        ret = false;
    }
    if (ay_bit < ST_MIN || ay_bit > ST_MAX) {
        println("AY Failed BIT");
        print("AY Bit Val: ");
        println(ay_bit);
        print("AY FT Val: ");
        println(ay_ft);
        print("AY STR Val: ");
        println(ay_st);
        ret = false;
    }
    if (az_bit < ST_MIN || az_bit > ST_MAX) {
        println("AZ Failed BIT");
        print("AZ Bit Val: ");
        println(az_bit);
        print("AZ FT Val: ");
        println(az_ft);
        print("AZ STR Val: ");
        println(az_st);
        ret = false;
    }
    if (gx_bit < ST_MIN || gx_bit > ST_MAX) {
        println("GX Failed BIT");
        print("GX Bit Val: ");
        println(gx_bit);
        print("GX FT Val: ");
        println(gx_ft);
        print("GX STR Val: ");
        println(gx_st);
        ret = false;
    }
    if (gy_bit < ST_MIN || gy_bit > ST_MAX) {
        println("GY Failed BIT");
        print("GY Bit Val: ");
        println(gy_bit);
        print("GY FT Val: ");
        println(gy_ft);
        print("GY STR Val: ");
        println(gy_st);
        ret = false;
    }
    if (gz_bit < ST_MIN || gz_bit > ST_MAX) {
        println("GZ Failed BIT");
        print("GZ Bit Val: ");
        println(gz_bit);
        print("GZ FT Val: ");
        println(gz_ft);
        print("GZ STR Val: ");
        println(gz_st);
        ret = false;
    }

    // Reset values to original state
    accelgyro.setFullScaleGyroRange(gyroVal);
    accelgyro.setFullScaleGyroRange(accelVal);
    return ret;
}

bool blinkState = false;

void SetupSDCard() {
    bool alreadyExists = false;
    println("Initializing SD card...");

    // Setup SD card for mkrzero board
    if(!SD.begin(SDCARD_SS_PIN)) {
        println("Card failed");
        exit(1);
    }

    if(SD.exists(filename)) {
        alreadyExists = true;
    }

    // Open up the file on the SD card
    datafile = SD.open(filename, FILE_WRITE);
    if(!datafile) {
        println("Failed to open file");
        exit(1);
    }

    if(alreadyExists) {
        datafile.println("\n\n\nNew Run\n\n\n");
    }
}

void setup() {
#if defined (__SERIAL_ONLY__) || defined (__BOTH_OUTPUTS__)
    // initialize serial communication
    Serial.begin(1000000);

    // Wait for communication to be up
    // Will need to be removed when running standalone
    while(!Serial);
#endif
#if defined (__CARD_ONLY__) || defined (__BOTH_OUTPUTS__)
    SetupSDCard();
#endif


    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize device
    println("Initializing I2C devices...");
    accelgyro.initialize();
    accelgyro.setFullScaleGyroRange(gyroRate);
    accelgyro.setFullScaleAccelRange(accelRate);

    // verify connection
    println("Testing device connections...");
    if(!accelgyro.testConnection()) {
        println("MPU6050 connection failed");
        //exit(1);
    }

    println("MPU6050 connection successful");

    // configure Arduino LED pin for output
    pinMode(LED_BUILTIN, OUTPUT);
    if(!selfTest()) {
        exit(1);
    }
    getBias();
    accelgyro.setXGyroOffset(121);
    accelgyro.setYGyroOffset(55);
    accelgyro.setZGyroOffset(54);
}

void loop() {
    print("a/g:\t");
    print("ax(g):\t");
    print("ay(g):\t");
    print("az(g):\t");
    print("gx:\t");
    print("gy:\t");
    println("gz:\t");
    for(int i = 0; i < 1000; i++) {
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // these methods (and a few others) are also available
        //accelgyro.getAcceleration(&ax, &ay, &az);
        //accelgyro.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        print("a/g:\t");
        print(ax/accelLsb[accelRate]); print("\t");
        print(ay/accelLsb[accelRate]); print("\t");
        print(az/accelLsb[accelRate]); print("\t");
        print((gx)/gyroLsb[gyroRate]); print("\t");
        print((gy)/gyroLsb[gyroRate]); print("\t");
        println((gz)/gyroLsb[gyroRate]);
        //print((gx-gxb)/gyroLsb[gyroRate]); print("\t");
        //print((gy-gyb)/gyroLsb[gyroRate]); print("\t");
        //println((gz-gzb)/gyroLsb[gyroRate]);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
        write((uint8_t)(ax >> 8)); write((uint8_t)(ax & 0xFF));
        write((uint8_t)(ay >> 8)); write((uint8_t)(ay & 0xFF));
        write((uint8_t)(az >> 8)); write((uint8_t)(az & 0xFF));
        write((uint8_t)(gx >> 8)); write((uint8_t)(gx & 0xFF));
        write((uint8_t)(gy >> 8)); write((uint8_t)(gy & 0xFF));
        write((uint8_t)(gz >> 8)); write((uint8_t)(gz & 0xFF));
#endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_BUILTIN, blinkState);
    }

    println("\n\n\nExiting\n\n\n");

    if(datafile) {
        datafile.close();
    }
    exit(0);
}
