#include <PID_v1.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter
#define LED 2
#include "BluetoothSerial.h"

//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;
BluetoothSerial my_bt;
char   control = 's';


AccelStepper motor_right(AccelStepper::DRIVER, 12, 13);
AccelStepper motor_left(AccelStepper::DRIVER, 23, 27);

double Kp = 152.5, Ki = 1575.0, Kd = .265;


/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle;  // Angle calculate using the gyro only
double compAngleX, compAngleY;  // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter
double pid_output, actual_op = 0, pid_input, setpoint = -1.87f , fixed_set_point , turn =0 ;
bool set = false;
PID myPid(&pid_input, &pid_output, &setpoint, Kp, Ki, Kd, DIRECT);

uint32_t timer;
uint8_t i2cData[14];  // Buffer for I2C data



hw_timer_t *My_timer = NULL;


void IRAM_ATTR onTimer() {
  

}

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();
  my_bt.begin("Self balance bot");

  // pinMode(LED, OUTPUT);
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 250000, true);
  timerAlarmEnable(My_timer);  //Just Enable


#if ARDUINO >= 157
  Wire.setClock(400000UL);  // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2;  // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7;     // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;  // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;  // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00;  // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false))
    ;  // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true))
    ;  // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1))
    ;
  if (i2cData[0] != 0x68) {  // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1)
      ;
  }

  delay(100);  // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH  // Eq. 25 and 26
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll);  // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  pinMode(2 , OUTPUT);
  setpoint = 0;

  for(int i = 0 ; i < 6 ;i++){
      digitalWrite(2 , i%2);
      delay(1000);
  }

  for(int i = 0 ; i < 1000 ; i++){
    setpoint +=setpoint_calc();
  }
  setpoint/=1000;
  fixed_set_point = setpoint;

  for(int i = 0 ; i < 6 ;i++){
      digitalWrite(2 , i%2);
      delay(400);
  }
  motor_right.setMaxSpeed(150000000000000);
  motor_left.setMaxSpeed(15000000000000000);
  motor_left.setAcceleration(2000000000000);
  motor_right.setAcceleration(20000000000000000);

  myPid.SetOutputLimits(-3250 , 3250);\
  
  myPid.SetMode(AUTOMATIC);

  timer = micros();
}


double setpoint_calc(){
  while (i2cRead(0x3B, i2cData, 14))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
  ;

  double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH  // Eq. 25 and 26
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0;  // Convert to deg/s
  double gyroYrate = gyroY / 131.0;  // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate;                           // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;

    return kalAngleX;
}

void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
  ;

  double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH  // Eq. 25 and 26
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0;  // Convert to deg/s
  double gyroYrate = gyroY / 131.0;  // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate;                           // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter
#endif

 
  pid_input = kalAngleX;
    if(my_bt.available()){
      control = my_bt.read();
      if(control == 'f' || control == 'b' || control == 's' || control == 'r')
      set = true;
    }

  
  if( control == 'f' && set){
      setpoint = fixed_set_point -1.62;
      set = false;
  }
 
  if( control == 'b' && set){
      setpoint = fixed_set_point +1.62;
      set = false;
  }

  if( control == 's' && set){
      setpoint = fixed_set_point;
      turn =0;
      set = false;
  }

  if( control == 'r' && set){
      setpoint = fixed_set_point;
      turn +=130;
      if (turn > pid_output) turn = 0;
      set = false;
  }


   myPid.Compute();

 
  

  
  motor_right.setSpeed(pid_output - turn);
    //motor_right.moveTo(test * (200.0 / 360));

  motor_left.setSpeed(-1*(pid_output+turn));

    //motor_left.moveTo(test * (200.0 / 360));

    motor_left.run();
    motor_right.run();

  




  /* Print Data */
  //Serial.println(kalAngleX);
}
