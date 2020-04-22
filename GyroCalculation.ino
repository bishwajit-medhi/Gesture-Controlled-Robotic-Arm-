/* 
    This is "Gesture Controlled Robotic" Arm code where you can control the Gyro Sensor in diffrent angle 
    This code will help to determine the angle of X,Y,Z axis of the gyro sensore. 
    Here we have take Flex sensore also to rotate the 4th servo
 */

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>

Servo servo1; // X-axis
Servo servo2; // Y-axis
Servo servo3; // Z-axis
Servo servo4;  //For Flex sensore

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Used to calculate the angle - Variables of the accelerometer
double accXangle;
double accYangle;
double accZangle;

// Used to calculate angle - Gyroscope variables (Note: For Diffrent sensore, The Offset value will be diffrent) 
double gyroXangle = 90;//Sensor readings with offsets:  -8  0 16388 -1  -1  0 offsets in raw values: -1165 6 1245  101 -19 71 
double gyroYangle = 115;
double gyroZangle = 125;

double gyroXrate;
double gyroYrate;
double gyroZrate;

double roll;
double pitch;
double yaw;

float previousTime;
float elapsedTime;
float currentTime;

double gyroXoffset;
double gyroYoffset;
double gyroZoffset;

uint32_t timer;

//flex pin read
int flexpin = A0; 
int flexvalue;
int servoposition= 0;

float x = 0, y = 0, z = 0;

int c=0;

void setup () {
 
  Wire.begin ();

  //Initializing serial communication
  Serial.begin (115200);

  // Starting devices
  Serial.println ("Initializing I2C ...");
  
  accelgyro.initialize ();
  //Testing the connection to the MPU6050
  Serial.println ("Testing the  MPU6050 connection ...");
  Serial.println (accelgyro.testConnection ()? "MPU6050 successfully connected": "Connection failed with MPU6050");

  while (c < 3000) {                                                    //Calculating the Servo for 3000 times to calculate the avegare drift

    accelgyro.getMotion6 (& ax, & ay, & az, & gx, & gy, & gz);

    x += ((float)gx) / 131.0;                                           
    y += ((float)gy) / 131.0;
    z += ((float)gz) / 131.0;
    c++;
  }

  //New offset Calculation to calibarate
  gyroXoffset = x / 3000; 
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;

  Serial.print("gyroXoffset : ");Serial.print(gyroXoffset);
  Serial.print("\ngyroYoffset : ");Serial.print(gyroYoffset);Serial.print ("\n");
  Serial.print("gyroZoffset : ");Serial.print(gyroZoffset);Serial.print ("\n");
  

  servo1.attach (9);    //Servo1 attached to 9 no Pin
  servo2.attach (10);   //Servo1 attached to 10 no Pin
  servo3.attach (11);   //Servo1 attached to 11 no Pin
  servo4.attach (6);    //Servo1 attached to 6 no Pin

  
  timer = millis ();
}

void loop () {
  // Reading the connection to the MPU6050
  accelgyro.getMotion6 (& ax, & ay, & az, & gx, & gy, & gz);
  

  // Calculate the angles based on the sensors
  double aX = (ax) / 16384.0; // +- 2 sensitivity
  double aY = (ay) / 16384.0;
  double aZ = (az) / 16384.0;
  accXangle = (atan2 (aY, aZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2 (aX, aZ) + PI) * RAD_TO_DEG;
  accZangle = (atan2 (aY, aZ) + PI) * RAD_TO_DEG;

   previousTime = currentTime;        // Previous time is stored before the actual time read
   currentTime = millis();            // Current time actual time read
   elapsedTime = (currentTime - previousTime) / 1000;

// gyrorate calculation
  gyroXrate = ((double) gx / 131.0);
  gyroYrate = ((double) gy / 131.0); 
  gyroZrate = ((double) gz / 131.0);

//gyrorate rate correction
   gyroXrate -= gyroXoffset;
   gyroYrate -= gyroYoffset;
   gyroZrate -= gyroZoffset;

  //calculate gyro angles
  gyroXangle += gyroXrate * elapsedTime;
  gyroYangle += gyroYrate * elapsedTime;
  gyroZangle += gyroZrate * elapsedTime;

  // apply complementry filter
  roll = (0.02 * accXangle) + (0.98 * gyroXangle);
  pitch = (0.02 * accYangle) + (0.98 * gyroYangle);
  yaw= gyroZangle;

  

  //Flex sensor reading
  flexvalue = analogRead(flexpin); 
  servoposition = map(flexvalue, 390, 500, 0, 180); 
  servoposition= constrain(servoposition, 0,180); 
 
  servo1.write (roll);
  servo2.write (pitch);
  servo3.write (gyroZangle);
  servo4.write (servoposition);

  timer = millis ();
  // The maximum sample rate of the accelerometer is 1KHz
  delay (1);

 // Serial.print (gyroXrate); Serial.print ("\t");
 // Serial.print (gyroYrate); Serial.print ("\t");
 // Serial.print (gyroZrate); Serial.print ("\t");
  // Angle x / y / z
  Serial.print ("X axis:");Serial.print (roll); Serial.print ("\t");
  Serial.print ("Y axis:");Serial.print (pitch); Serial.print ("\t");
  Serial.print ("Z axis:");Serial.print (yaw); Serial.print ("\t");
  Serial.print(flexvalue);Serial.print ("\t");
    
  Serial.print ("\n");
}
