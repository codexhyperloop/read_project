#include <SPI.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_LSM9DS0.h"

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections

   (For Udoo Quad I2C)
   ===========
   Connect SCL to pin 21
   Connect SDA to pin 20
   Connect VDD to 3V3
   Connect GROUND to common ground
   ===========
   
   (For default I2C)
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground
   ===========
   
   (For SPI)
   ===========
   Connect SCL to 
   History
   =======
   2014/JULY/25  - First version (KTOWN)
*/
   
// ------------------------------------------------------------------------------------------
// Variables
// ------------------------------------------------------------------------------------------
/* Assign a unique base ID for this sensor */   
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
// ------------------------------------------------------------------------------------------


/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 4
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

//#define LSM9DS0_SCLK 142
//#define LSM9DS0_MISO 144
//#define LSM9DS0_MOSI 143

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);


//bool badRoundRun = LOW; // <- The first run through of the accelerometer gives bad readings,
            // so we run once without printing values, then flip this to high so
            // we print values from that point on
sensors_event_t accel, accelSum, prevAccel;
float timeDif;

/**************************************************************************
    Arduino setup function (automatically called at startup)
**************************************************************************/
void setup(void) 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
  delay(1000);

  // Find the average error over 100 samples and store it for calibration
  accelSum.acceleration.x = 0;
  accelSum.acceleration.y = 0;
  accelSum.acceleration.z = 0;
  for (uint16_t counter = 0; counter < 100; counter++) {
    lsm.readAccel();
    lsm.getAccelEvent(&accel, millis());
    accelSum.acceleration.x += accel.acceleration.x;
    accelSum.acceleration.y += accel.acceleration.y;
    accelSum.acceleration.z += accel.acceleration.z;
    /*Serial.print("Prev Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");  //Used for debugging. Uncomment to see the accel values and sum values throughout the calibration process
    Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
    Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");

    Serial.print("SumAccel X: "); Serial.print(accelSum.acceleration.x); Serial.print(" ");
    Serial.print("  \tY: "); Serial.print(accelSum.acceleration.y);       Serial.print(" ");
    Serial.print("  \tZ: "); Serial.print(accelSum.acceleration.z);     Serial.println("  \tm/s^2");
    */
  }

  accelSum.acceleration.x /= 100;
  accelSum.acceleration.y /= 100;
  accelSum.acceleration.z /= 100;

  lsm.accelCalibration.x = accelSum.acceleration.x;
  lsm.accelCalibration.y = accelSum.acceleration.y;
  lsm.accelCalibration.z = accelSum.acceleration.z;

  /*Serial.print("SumAccel X: "); Serial.print(accelSum.acceleration.x); Serial.print(" "); //Used for debugging. Uncomment to see the Sum for calibration process.
  Serial.print("  \tY: "); Serial.print(accelSum.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accelSum.acceleration.z);     Serial.println("  \tm/s^2");
  */
  Serial.print("AvgAccel X: "); Serial.print(accelSum.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(accelSum.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accelSum.acceleration.z);     Serial.println("  \tm/s^2");
  
}

/**************************************************************************
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
**************************************************************************/
void loop() 
{ 
  /* For some reason the first reading is bad, so run
  // it once and move on with the loop as normal
  if (!badRoundRun) {
    lsm.readAccel();
    lsm.getAccelEvent(&accel, millis());
    badRoundRun = HIGH;  
  } */
  
  /*// print out previous acceleration data  
  Serial.print("  \nTime: "); Serial.println(prevAccel.timestamp);
  Serial.print("Prev Accel X: "); Serial.print(prevAccel.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(prevAccel.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(prevAccel.acceleration.z);     Serial.print(" ");
  Serial.print("\tMagnitude:"); Serial.print(accel.acceleration.magnitude); Serial.println("  \tm/s^2");
  */

  //This loop is used to get values from multiple integrations without having to use the slow Serial.print functions
  for (uint32_t counter = 0; counter < 10; counter++) {
    // ---- Get new data ----
    lsm.readAccel();
    lsm.getAccelEvent(&accel, micros());
    timeDif = accel.timestamp - prevAccel.timestamp;
    lsm.calcVelocity(&prevAccel, &accel, timeDif);
    lsm.calcDistance(timeDif);
    prevAccel = accel;
    // ----------------------
    delayMicroseconds(1);
  }
  
  // print acceleration data
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.print(" ");
  Serial.print("\tMagnitude:"); Serial.print(accel.acceleration.magnitude); Serial.println("  \tm/s^2");
  
  // print velocity data
  Serial.print("Velocity X: "); Serial.print(lsm.velocity.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(lsm.velocity.y); Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(lsm.velocity.z); Serial.print(" ");
  Serial.print("\tMagnitude:"); Serial.print(lsm.velocity.magnitude); Serial.println("  \tm");

  // print distance data
  Serial.print("Distance X: "); Serial.print(lsm.distance.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(lsm.distance.y); Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(lsm.distance.z); Serial.print(" ");
  Serial.print("\tMagnitude:"); Serial.print(lsm.distance.magnitude); Serial.println("  \tm");
  
  // print the time differential from the last loop to now
  Serial.print("  \nTimeDif: "); Serial.print(timeDif);
  Serial.println("\n**********************\n");

  delay(250);
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  delay(2000);

  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  //These aren't used curently. No point in displaying details.
  /*Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  */
  
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the LSM9DS0
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);  //Our pod will be experiencing more than 2Gs but less than 4Gs
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}
