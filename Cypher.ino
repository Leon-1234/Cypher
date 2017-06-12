

/*
 * Sensors work fine as of now. PID controllers for all three main axes seems to be working fine right now.
 * Only simulator tests done so far. Working on a controller right now.
 * 
 */


#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

#define MPU6050 0x68   // Address of MPU6050 Accelerometer Gyroscope. 0x69 if AD0 is set to HIGH on the PCB
#define HMC5883L 0x1E  // Address of HMC5883L Magnetometer
#define BMP180 0x77    // Address of BMP180 Pressure and Temperature Sensor

//#define OUTPUT_COMP_YPR // Set Complimentary Filter Output
#define OUTPUT_MOTOR

#define I2C_TIMEOUT 1000
#define BARO_POWER 0.1902949

#define M_FR_MIN 1200   // Minimum Pulse for Arming the Front-Right ESC
#define M_FL_MIN 1200   // Minimum Pulse for Arming the Front-Left ESC
#define M_RR_MIN 1200   // Minimum Pulse for Arming the Rear-Right ESC
#define M_RL_MIN 1200   // Minimum Pulse for Arming the Rear-Right ESC

// For Motor Pins
int motorFR_Pin = 3; 
int motorFL_Pin = 4; 
int motorRR_Pin = 5; 
int motorRL_Pin = 6;

// For Raw IMU Data 
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;
double temp, heading;

double roll, pitch, yaw; // Roll and Pitch are calculated from Accelerometer + Gyro and Yaw is calculated from Magnetometer + Gyro

double compAngleX, compAngleY, compAngleZ; // Estimated angles using a Complementary filter. 

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// Calibration Values for the BMP180 Sensor. Taken from the registors of the sensor.
int AC1;
int AC2;
int AC3;
unsigned int AC4;
unsigned int AC5;
unsigned int AC6;
int B;
int B2;
int MB;
int MC;
int MD;

// Pressure Temperature and Altitude from Barometric Pressure Sensor
float BMP_P;
double BMP_T, BMP_A;

// Output values from the PID controllers
double pidAngleX, pidAngleY, pidAngleZ;

// Input from Reciever
int X1 = 512, X2 = 512, Y1 = 512, Y2 = 512;
int B_1 = 1, B_2 = 1, B_3 = 1, B_4 = 1;
int UP = 1, DOWN = 1, LEFT = 1, RIGHT = 1;

// Control Variables
int throttle = 0;
uint8_t isArmed = 0;

// Setpoints for the PID Controller
double setX = 0, setY = 0, setZ = 0;

// Tunings for the PID Controller
double Kp = 2;
double Ki = 0.02;
double Kd = 0.015;

// Raw Motor Pulses
int motorFR = 0, motorFL = 0, motorRR = 0, motorRL = 0;

// Instances of the Servo class for the ESCs
Servo M_FR;
Servo M_FL;
Servo M_RR;
Servo M_RL;

// Setup the PID controllers.
PID pitchPID(&compAngleX, &pidAngleX, &setX, Kp, Ki, Kd, DIRECT);
PID rollPID(&compAngleY, &pidAngleY, &setY, Kp, Ki, Kd, DIRECT);
PID yawPID(&compAngleZ, &pidAngleZ, &setZ, 1, 0.02, 0.015, DIRECT);


void motorWrite(int);

void setup() {

  delay(100); // Wait for Sensors to initialize

  Serial.begin(57600);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400Hz

  calibrateMPU6050();
  calibrateHMC5883L();
  calibrateBMP();
  
  delay(100); // Wait for Sensors to stabilize

  //Initialize all values
  
  updateMPU6050();	
  updateHMC5883L(); 
  updateYPR();
    
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = yaw;

  timer = micros(); // Initialize the timer

  M_FR.attach(motorFR_Pin);
  M_FL.attach(motorFL_Pin);
  M_RR.attach(motorRR_Pin);
  M_RL.attach(motorRL_Pin);

  initPID(); 
  
  delay(100);
    
}

void loop() {
  
  // Get Raw data from sensors
  
  updateMPU6050();
  updateHMC5883L();
  updateBMP();

  // Yaw Pitch Roll Estimation 
  
  updateYPR();

  // Use a Complimentary Filter
  
  compYPR();

  cmdRead();
  updateJoysticks();
  
  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();

  if(isArmed == 2) {
    motorWrite(1);
    isArmed = 1;
  }
  if(isArmed == 1) motorWrite(0);

  #ifdef OUTPUT_PID
  
    Serial.print(setX);
    Serial.print('\t');
    Serial.print(setY);
    Serial.print('\t');
    Serial.print(setZ);
    Serial.print('\t');
    Serial.print(pidAngleX);
    Serial.print('\t');
    Serial.print(pidAngleY);
    Serial.print('\t');
    Serial.print(pidAngleZ);
    Serial.print('\t');
    
  #endif
  
  Serial.println(); 
  //delay(10);
  
}

// This function is used to read and process the command form the controller
void cmdRead() {

  String cmd;
  int temp;
  while (Serial.available() > 0) {
    
     cmd = Serial.readStringUntil('%');
     //Serial.println(cmd); 
     if(cmd.startsWith("#")  && cmd.endsWith("!") && cmd.length() == 21 ) {  
         
         
         //Serial.println("");
         //Serial.println(cmd);
         Y2 = (cmd.substring( 1, 5)).toInt();
         X1 = (cmd.substring(5,9)).toInt();
         X2 = (cmd.substring(9,13)).toInt();
         Y1 = (cmd.substring(13,17)).toInt();
         temp = (cmd.substring(17,20)).toInt();

         B_2 = bitRead(temp,0);
         B_3 = bitRead(temp,1);
         B_4 = bitRead(temp,2);
         B_1 = bitRead(temp,3);
         LEFT = bitRead(temp,4);
         RIGHT = bitRead(temp,5);
         DOWN = bitRead(temp,6);
         UP = bitRead(temp,7);   

         //printcmd();
      
     }  
     
     Serial.flush();
     
  }
  
}

// This function is used to debug controller
void printcmd() {

   Serial.print(X1);
   Serial.print('\t');
   Serial.print(Y1);
   Serial.print('\t');
   Serial.print(X2);
   Serial.print('\t');
   Serial.print(Y2);
   Serial.print('\t');
   Serial.print(B_1);
   Serial.print('\t');
   Serial.print(B_2);
   Serial.print('\t');
   Serial.print(B_3);
   Serial.print('\t');
   Serial.print(B_4);
   Serial.print('\t');
   Serial.print(UP);
   Serial.print('\t');
   Serial.print(DOWN);
   Serial.print('\t');
   Serial.print(LEFT);
   Serial.print('\t');
   Serial.print(RIGHT);
   Serial.print('\t');
         
}

// This function passes the roll, pitch and yaw values through a complimentary filter

void compYPR() {

  // Calculate the time elapsed for the Complimentary Filter
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();
  
  // Calculate gyroRates in deg/s for the Complementary Filter
  // We have set the full scale range of the gyro to +/- 250deg/s
  // So from datasheet we have 131 LSB/(deg/s) for the range of +/- 250deg/s
  
  double gyroXrate = gyroX / 131.0;   
  double gyroYrate = gyroY / 131.0; 
  double gyroZrate = gyroZ / 131.0;

  // Reset Complementary Filter when angles jump between -180 and 180 to smooth the transition
  if ((yaw < -90 && compAngleZ > 90) || (yaw > 90 && compAngleZ < -90))    compAngleZ = yaw;
  if ((pitch < -90 && compAngleY > 90) || (pitch > 90 && compAngleY < -90)) compAngleY = pitch;
  if ((roll < -90 && compAngleX > 90) || (roll > 90 && compAngleX < -90)) compAngleX = roll;
    
  // Estimate angles using a Complimentary Filter
  
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  #ifdef OUTPUT_COMP_YPR
  
    Serial.print(compAngleX); 
    Serial.print("\t");
    Serial.print(compAngleY); 
    Serial.print("\t");
    Serial.print(compAngleZ); 
    Serial.print("\t");

  #endif
  
}

void updateMPU6050() {
  
  while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Read the 14 MPU6050 data-out registers one by one
  
  accX  = ((i2cData[0] << 8) | i2cData[1]);
  accY  = -((i2cData[2] << 8) | i2cData[3]);
  accZ  = ((i2cData[4] << 8) | i2cData[5]);
  
  temp  = (i2cData[6] << 8) | i2cData[7];    // MPU6050 core temperature
  
  gyroX = -(i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = -(i2cData[12] << 8) | i2cData[13];

  // Convert MPU6050 die core Temperature to Celsius
  temp = temp / 340.0 + 36.53;
  
  #ifdef OUTPUT_ACCGYRO_RAW
  
    Serial.print(" AccX: ");
    Serial.print(accX);
    Serial.print(" AccY: ");
    Serial.print(accY);
    Serial.print(" AccZ: ");
    Serial.print(accZ);
    Serial.print("\t");
    Serial.print(" GyroX: ");
    Serial.print(gyroX);
    Serial.print(" GyroY: ");
    Serial.print(gyroY);
    Serial.print(" GyroZ: ");
    Serial.print(gyroZ);
    Serial.print("\t");
    
  #endif

  #ifdef OUTPUT_ACC_G 
  
    Serial.print(accX / 16384.0); Serial.print("\t"); 
    Serial.print(accY / 16384.0); Serial.print("\t");
    Serial.print(accZ / 16384.0); Serial.print("\t");
    
  #endif

  #ifdef OUTPUT_GYRO_DEG
  
    Serial.print(gyroX / 131); Serial.print("\t"); 
    Serial.print(gyroY / 131); Serial.print("\t");
    Serial.print(gyroZ / 131); Serial.print("\t");
    
  #endif

  #ifdef OUTPUT_TEMPERATURE 
  
    Serial.print("\t");
    Serial.print(temp); 
    Serial.print("\t");
    
  #endif
  
}

// Set up the PID controllers

void initPID() {

  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-50,50);
  pitchPID.SetSampleTime(15);

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-70,70);
  rollPID.SetSampleTime(15);
  
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-50,50);
  yawPID.SetSampleTime(15);
  
}

void motorWrite(int esc_min = 0) {

  motorFR = M_FR_MIN + throttle - pidAngleX + pidAngleY - pidAngleZ;
  motorFL = M_FL_MIN + throttle - pidAngleX - pidAngleY + pidAngleZ;
  motorRR = M_RR_MIN + throttle + pidAngleX + pidAngleY + pidAngleZ;
  motorRL = M_RL_MIN + throttle + pidAngleX - pidAngleY - pidAngleZ;

  if(motorFR < M_FR_MIN) motorFR = M_FR_MIN;
  if(motorFL < M_FL_MIN) motorFL = M_FL_MIN;
  if(motorRR < M_RR_MIN) motorRR = M_RR_MIN;
  if(motorRL < M_RL_MIN) motorRL = M_RL_MIN;
  
  else if(motorFR > 2300) motorFR = 2300;
  else if(motorFL > 2300) motorFL = 2300;
  else if(motorRR > 2300) motorRR = 2300;
  else if(motorRL > 2300) motorRL = 2300;

  if(esc_min == 1) {        // ESC should recieve a 1100 pulse to start up.
    
    motorFR = 1100;
    motorFL = 1100;
    motorRR = 1100;
    motorRL = 1100;
    
  }

  M_FR.write(motorFR);
  M_FL.write(motorFL);
  M_RR.write(motorRR);
  M_RL.write(motorRL);

  #ifdef OUTPUT_MOTOR

    Serial.print(motorFR);
    Serial.print('\t');
    Serial.print(motorFL);
    Serial.print('\t');
    Serial.print(motorRR);
    Serial.print('\t');
    Serial.print(motorRL);
    Serial.print('\t');

  #endif
  
}

// Read Analog Values from Joysticks

void updateJoysticks() {

  setX = map(X1, 0, 1023, -50, 50);
  setY = map(Y1, 0, 1023, -50, 50);
  setZ = map(Y2, 0, 1023, -30, 30);

  if(X1 == 517) setX = 0;
  if(Y1 == 508) setY = 0;
  if(Y2 == 511) setZ = 0;
  
  if(X2 > 800) throttle = throttle-5;
  else if(X2 < 300) throttle = throttle+5;
  if(throttle < 0) throttle = 0;
  if(throttle > 800) throttle = 800; 

  if(B_3 == 0 && B_4 == 0) {
    
    if(isArmed == 0) isArmed = 2;
    else isArmed = 0;
    
  }
  
  #ifdef OUTPUT_JOYSTICK
   
    Serial.print(X1);
    Serial.print('\t');
    Serial.print(Y1);
    Serial.print('\t');
    Serial.print(X2);
    Serial.print('\t');
    Serial.print(Y2);
    Serial.print('\t');
    Serial.print(throttle);
    Serial.print('\t');
    Serial.print(setZ);
    Serial.print('\t');
    Serial.print(setX);
    Serial.print('\t');
    Serial.print(setY);
    Serial.println('\t');

  #endif  
  
}

void updateHMC5883L() {
  
  while (i2cRead(HMC5883L, 0x03, i2cData, 6)); // Get magnetometer values
  magY = ((i2cData[0] << 8) | i2cData[1]);
  magZ = ((i2cData[2] << 8) | i2cData[3]);
  magX = ((i2cData[4] << 8) | i2cData[5]);
  
  #ifdef OUTPUT_MAG_RAW
  
    Serial.print(" MagX: ");
    Serial.print(magX);
    Serial.print(" MagY: ");
    Serial.print(magY);
    Serial.print(" MagZ: ");
    Serial.print(magZ);
    Serial.print("\t");
    
  #endif
  
}

void updateBMP() {
 
  long uTemp, uPressure;
  
  long T1, T2, T3, B3, B5, B6, p;
  unsigned long B4, B7;
  
  uint8_t OSS = 3;  // Sampling Resolution
  
  i2cWrite(BMP180, 0xF4, 0x2E, true);
  delay(5); // 4.5ms Minimum delay for sampling from datasheet.
  while (i2cRead(BMP180, 0xF6, i2cData, 2));
  
  uTemp = i2cData[0] << 8 | i2cData[1]; 
  
  i2cWrite(BMP180,0xF4, 0x34+(OSS<<6), true);
  delay(2 + (3<<OSS));
  while (i2cRead(BMP180, 0xF6, i2cData, 3));
  
  uPressure = (((unsigned long) i2cData[0] << 16) | ((unsigned long) i2cData[1] << 8) | (unsigned long) i2cData[2]) >> (8-OSS);
  
  T1 = (((long)uTemp - (long)AC6) * (long)AC5 >> 15);
  T2 =  ((long)MC << 11) / (T1 + MD); 
  B5 = T1 + T2;
  BMP_T = (B5 + 8) >> 4;
  BMP_T /= 10;
  
   B6 = B5 - 4000;
  // Calculate B3
  T1 = (B2 * (B6 * B6)>>12)>>11;
  T2 = (AC2 * B6)>>11;
  T3 = T1 + T2;
  B3 = (((((long)AC1)*4 + T3)<<OSS) + 2)>>2;

  T1 = (AC3 * B6)>>13;
  T2 = (B * ((B6 * B6)>>12))>>16;
  T3 = ((T1 + T2) + 2)>>2;
  B4 = (AC4 * (unsigned long)(T3 + 32768))>>15;

  B7 = ((unsigned long)(uPressure - B3) * (50000>>OSS));
  if (B7 < 0x80000000)
    p = (B7<<1)/B4;
  else
    p = (B7/B4)<<1;

  T1 = (p>>8) * (p>>8);
  T1 = (T1 * 3038)>>16;
  T2 = (-7357 * p)>>16;
  p += (T1 + T2 + 3791)>>4;
  BMP_P = p;
  
  double atmosPressure = (float)p/101325;
  double T5 = pow(atmosPressure,BARO_POWER);
  T5 = (1 - T5);
  BMP_A = T5 * 44300;

  #ifdef OUTPUT_BMP
  
    Serial.print("Temperature : ");
    Serial.print(BMP_T); 
    Serial.print("  Pressure : ");
    Serial.print(p);
    Serial.print("  Standard : ");
    Serial.print(atmosPressure);
    Serial.print("  Altitude : ");
    Serial.println(BMP_A);
    
  #endif
  
} 

void updateYPR() {

  roll = atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  // We have set a gain of 1.3gauss for the Magnetometer. 
  // So we apply the gain values from the datasheet here.
  
  magX *= 0.92;
  magY *= 0.92;
  magZ *= 0.92;

  double rollAngle = compAngleX * DEG_TO_RAD;
  double pitchAngle = compAngleY * DEG_TO_RAD;
  
  double Bfx = magX * cos(pitchAngle) + magZ * sin(pitchAngle);
  double Bfy = magX * sin(rollAngle) * sin(pitchAngle) + magY * cos(rollAngle) - magZ * sin(rollAngle) * cos(pitchAngle);
  yaw = (atan2(-Bfy, Bfx) * RAD_TO_DEG);


  yaw += 108;                             //Applying Offsets
  if(yaw > 180) yaw = (360 - yaw)* -1;    //Fixing Quadrants
  
  heading = atan2(-magY,magX);
  float declinationAngle = (8.0 + (2.0 / 60.0)) / (180 / M_PI);
  heading -= declinationAngle;
  heading *= RAD_TO_DEG;
  if(heading > 180) heading -= 180;
  if(heading < 0) heading += 180;
  
  #ifdef OUTPUT_YPR
  
    Serial.print(compAngleX); 
    Serial.print("\t");
    Serial.print(compAngleY); 
    Serial.print("\t");
    Serial.print(compAngleZ); 
    Serial.print("\t");
    Serial.print(" H: ");
    Serial.print(heading);

  #endif
  
}

// Function acquires calibration offsets stored in BMP180's registors. Refer to datasheet for more details on these offsets.

void calibrateBMP() {

  while (i2cRead(BMP180, 0xAA, i2cData, 6));
  AC1 = i2cData[0] << 8 | i2cData[1];
  AC2 = i2cData[2] << 8 | i2cData[3];
  AC3 = i2cData[4] << 8 | i2cData[5];
  
  while (i2cRead(BMP180, 0xB0, i2cData, 10));
  AC4 = i2cData[0] << 8 | i2cData[1];
  AC5 = i2cData[2] << 8 | i2cData[3];
  AC6 = i2cData[4] << 8 | i2cData[5];
  B = i2cData[6] << 8 | i2cData[7];
  B2 = i2cData[8] << 8 | i2cData[9];
  
  while (i2cRead(BMP180, 0xBA, i2cData, 6));
  MB = i2cData[0] << 8 | i2cData[1];
  MC = i2cData[2] << 8 | i2cData[3];
  MD = i2cData[4] << 8 | i2cData[5];
    
}

// This function calibrates the MPU6050 for sampling. Call this in setup before using the sensor.

void calibrateMPU6050() {

  i2cData[0] = 7;     // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;  // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;   // Set Gyro Full Scale Range to +/- 250deg/s
  i2cData[3] = 0x00;  // Set Accelerometer Full Scale Range to +/- 2g
  
  while (i2cWrite(MPU6050, 0x19, i2cData, 4, false)); // Configure Settings for the MPU6050
  while (i2cWrite(MPU6050, 0x6B, 0x01, true)); 
  
}

// This function sets the HMC5883L into continuous measuring mode. Call this in setup before using the sensor.

void calibrateHMC5883L() {

  while (i2cWrite(HMC5883L, 0x02, 0x00, true));   // Configure HMC5883L for continuous mode for calibration
  
}


uint8_t i2cWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data, uint8_t length, bool releaseBus) {
  
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t errCode = Wire.endTransmission(releaseBus);
  return errCode;
  
}

uint8_t i2cWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data, bool releaseBus) {
  
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  uint8_t errCode = Wire.endTransmission(releaseBus);
  return errCode;
  
}


uint8_t i2cRead(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  
  uint32_t timeOutTimer;
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  uint8_t errCode = Wire.endTransmission(false); // Don't release the bus
  if (errCode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(errCode);
    return errCode; 
  }
  Wire.requestFrom(deviceAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        return 5; // Read Timeout
      }
    }
  }
  return 0; 
  
}




