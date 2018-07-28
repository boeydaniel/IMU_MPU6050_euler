/*
===Contact & Support===
Website: http://eeenthusiast.com/
Youtube: https://www.youtube.com/EEEnthusiast
Facebook: https://www.facebook.com/EEEnthusiast/
Patreon: https://www.patreon.com/EE_Enthusiast
Revision: 1.0 (July 13th, 2016)
===Hardware===
- Arduino Uno R3
- MPU-6050 (Available from: http://eeenthusiast.com/product/6dof-mpu-6050-accelerometer-gyroscope-temperature/)
===Software===
- Latest Software: https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
- Arduino IDE v1.6.9
- Arduino Wire library
===Terms of use===
The software is provided by EEEnthusiast without warranty of any kind. In no event shall the authors or 
copyright holders be liable for any claim, damages or other liability, whether in an action of contract, 
tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in 
the software.
*/

// SDA A4 SCL A5

#include <Wire.h>
#include <math.h>
//#include "MegunoLink.h"
//#include "Filter.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//ExponentialFilter<long> GyroXFilter(70, 0);

float Rx, Ry, Rz;
float Gx, Gy, Gz, GyGz_magnitude;

float accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ, gForceXY_magnitude, gForceX1, gForceY1, gForceZ1;
float gForceX_cal, gForceY_cal;

float gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ, rotX1, rotY1, rotZ1;
float rotX_cal,rotY_cal,rotZ_cal;
float rotX_smooth, rotY_smooth, rotZ_smooth, rotX1_prev =0, rotY1_prev=0, rotZ1_prev=0;
float rotX1_prev2, rotY1_prev2, rotZ1_prev2;
float rotX_smooth1, rotY_smooth1, rotZ_smooth1;

uint32_t loop_timer;
float angle_pitch, angle_roll, angle_yaw;

void setup() {
  
  Serial.begin(9600);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(30,0);
  display.println("Calibrating");
  display.setCursor(15,15);
  display.println("Hold Sensor Still");
  display.display();
  setupMPU();
  calibrateGyro();
  intitialG();
  calibrateAcc_XY();
  calibrateAcc_Z();

  
  display.setTextColor(WHITE);
  display.setCursor(15,0);
  display.println("Calibration Done");
  display.display();
  delay(5000);
  loop_timer = micros();
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  rotationMatrix();
  exponentialSmoothing();
  
  angle_pitch += (float)rotX_smooth1 * (micros()-loop_timer)*0.000001;                                    
  angle_roll += (float)rotY_smooth1 * (micros()-loop_timer)*0.000001;
  angle_yaw += (float)rotZ_smooth1 * (micros()-loop_timer)*0.000001; 
  
  loop_timer = micros();
  
  printData();  
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void rotationMatrix()
{
 // gForceX = (cos(Rz)*cos(Ry)*gForceX)+(((cos(Rz)*sin(Rx)*sin(Ry))-(cos(Rx)*sin(Rz)))*gForceY)+(((sin(Rx)*sin(Rz))+(cos(Rx)*cos(Rz)*sin(Ry)))*gForceZ);
 // gForceY = (cos(Ry)*sin(Rz)*gForceX)+(((cos(Rx)*cos(Rz))+(sin(Rx)*sin(Rz)*sin(Ry)))*gForceY)+(((cos(Rx)*sin(Rz)*sin(Ry))-(cos(Rz)*sin(Rx)))*gForceZ);
 // gForceZ = (-sin(Ry)*gForceX)+(cos(Ry)*sin(Rx)*gForceY)+(cos(Rx)*cos(Ry)*gForceZ);

  //gForceX = (cos(Rz)*cos(Ry)*gForceX)+(cos(Ry)*sin(Rz)*gForceY)+(-sin(Ry)*gForceZ);
  //gForceY = (((cos(Rz)*sin(Rx)*sin(Ry))-(cos(Rx)*sin(Rz)))*gForceX)+(((cos(Rx)*cos(Rz))+(sin(Rx)*sin(Rz)*sin(Ry)))*gForceY)+(cos(Ry)*sin(Rx)*gForceZ);
  //gForceZ = (((sin(Rx)*sin(Rz))+(cos(Rx)*cos(Rz)*sin(Ry)))*gForceX)+(((cos(Rx)*sin(Rz)*sin(Ry))-(cos(Rz)*sin(Rx)))*gForceY)+(cos(Rx)*cos(Ry)*gForceZ);
 
  gForceX1 = (cos(Rz)*cos(Ry)*gForceX)+(-cos(Ry)*sin(Rz)*gForceY)+(-sin(Ry)*gForceZ);
  gForceY1 = (((-cos(Rz)*sin(Rx)*sin(Ry))+(cos(Rx)*sin(Rz)))*gForceX)+(((cos(Rx)*cos(Rz))+(sin(Rx)*sin(Rz)*sin(Ry)))*gForceY)+(-cos(Ry)*sin(Rx)*gForceZ);
  gForceZ1 = (((sin(Rx)*sin(Rz))+(cos(Rx)*cos(Rz)*sin(Ry)))*gForceX)+(((-cos(Rx)*sin(Rz)*sin(Ry))+(cos(Rz)*sin(Rx)))*gForceY)+(cos(Rx)*cos(Ry)*gForceZ);

  //gForceX1 = (cos(Ry)*gForceX)+(-sin(Ry)*gForceZ);
  //gForceY1 = (-sin(Rx)*sin(Ry)*gForceX)+(cos(Rx)*gForceY)+(-cos(Ry)*sin(Rx)*gForceZ);
  //gForceZ1 = (cos(Rx)*sin(Ry)*gForceX)+(sin(Rx)*gForceY)+(cos(Rx)*cos(Ry)*gForceZ);
  
  //gForceX = (cos(Rz)*gForceX1)-(sin(Rz)*gForceY1);
  //gForceY = (sin(Rz)*gForceX1)+(cos(Rz)*gForceY1);
  //gForceZ = gForceZ1;
  
  //gForceX = cos(Rz)*(sqrt(pow(gForceX,2)+pow(gForceY,2)));
  //gForceY = sin(Rz)*(sqrt(pow(gForceX,2)+pow(gForceY,2)));

  
  //offset gyro with initial reading
  rotX -= rotX_cal;
  rotY -= rotY_cal;
  rotZ -= rotZ_cal;
  
  float w;

  if(rotX_smooth <=0.2 && rotY_smooth <=0.2 && rotZ_smooth <=0.2) w=0.1;
  else w = 0.6;
  
  rotX_smooth = w*rotX+((1-w)*rotX1_prev);
  rotY_smooth = w*rotY+((1-w)*rotY1_prev);
  rotZ_smooth = w*rotZ+((1-w)*rotZ1_prev);

  rotX1_prev = rotX_smooth;
  rotY1_prev = rotY_smooth;
  rotZ1_prev = rotZ_smooth;
 
  rotX1 = rotX_smooth+(rotY_smooth*sin(Rx)*tan(Ry))+(rotZ_smooth*cos(Rx)*tan(Ry));
  rotY1 = (rotY_smooth*cos(Rx))-(rotZ_smooth*sin(Rx));
  rotZ1 = (rotY_smooth*sin(Rx)/cos(Ry))+(rotZ_smooth*cos(Rx)/cos(Ry));
}

//smooth out reading again after rotation transformation of gyro
void exponentialSmoothing()
{
  
// GyroXFilter.Filter(rotX1);

//  TimePlot Plot;
//  Plot.SendData("Raw", rotX1);
//  Plot.SendData("Filtered", GyroXFilter.Current());

  float w;
  
  if(rotX_smooth1 <=0.2 && rotY_smooth1 <=0.2 && rotZ_smooth1 <=0.2) w=0.1;
  else w = 0.6;
  
  rotX_smooth1 = w*rotX1+((1-w)*rotX1_prev2);
  rotY_smooth1 = w*rotY1+((1-w)*rotY1_prev2);
  rotZ_smooth1 = w*rotZ1+((1-w)*rotZ1_prev2);

  rotX1_prev2 = rotX_smooth1;
  rotY1_prev2 = rotY_smooth1;
  rotZ1_prev2 = rotZ_smooth1;
}

void printData() {

/*  
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX1);
  Serial.print(" Y=");
  Serial.print(rotY1);
  Serial.print(" Z=");
  Serial.print(rotZ1);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
*/
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20,0);
  display.print("x     y     z");
  display.setCursor(0,10);
  display.print("A ");
  display.print(gForceX1);
  display.print("  ");
  display.print(gForceY1);
  display.print("  ");
  display.println(gForceZ1);
  
  display.setCursor(0,25);
  display.print("G ");
  display.print(angle_pitch);
  display.print("  ");
  display.print(angle_roll);
  display.print("  ");
  display.println(angle_yaw);
  display.display();

}

//get initial static gyro reading
void calibrateGyro()
{
  for(int count = 0; count <2000; count++)
  {
    recordGyroRegisters();
    rotX_cal += rotX;
    rotY_cal += rotY;
    rotZ_cal += rotZ;
    //delay(4);
  }

  rotX_cal /=2000;
  rotY_cal /=2000;
  rotZ_cal /=2000;
}

// get initial static accelerometer reading
void intitialG()
{
  for(int count = 0; count <2000; count++)
  {
    recordAccelRegisters();
    Gx += gForceX;
    Gy += gForceY;
    Gz += gForceZ;
    //delay(4);
  }

  Gx /=2000;
  Gy /=2000;
  Gz /=2000;
  recordAccelRegisters();
  /*
  Serial.println("Initial Acc");
  Serial.print(gForceX); Serial.print("\t");
  Serial.print(gForceY); Serial.print("\t");
  Serial.println(gForceZ);
  Serial.println("Gx Gy Gz");
  Serial.print(Gx); Serial.print("\t");
  Serial.print(Gy); Serial.print("\t");
  Serial.println(Gz); */
}

//get roll & pitch correction angle
void calibrateAcc_XY()
{
  Rx = atan2(Gy,Gz);
  GyGz_magnitude = pow((pow(Gy,2)+pow(Gz,2)),0.5);
  Ry = atan2(Gx,GyGz_magnitude);

/*
  Serial.println("Rx Ry");
  Serial.print(Rx); Serial.print("\t");
  Serial.println(Ry);*/
}


//get yaw correction angle
void calibrateAcc_Z()
{

  
  for(int count = 3; count>0; count--)
  {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.setCursor(20,0);
    display.print("Move forward in");
    display.setCursor(60,15);
    display.setTextSize(2);
    display.print(count);
    display.display();
    delay(1000);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(30,15);
  display.print("Calibrating");
  display.display();
  
  for(int count = 0; count <1000; count++)
  {
    recordAccelRegisters();
    gForceX1 = (cos(Ry)*gForceX)+(-sin(Ry)*gForceZ);
    gForceY1 = (-sin(Rx)*sin(Ry)*gForceX)+(cos(Rx)*gForceY)+(-cos(Ry)*sin(Rx)*gForceZ);

    gForceX_cal += gForceX1;
    gForceY_cal += gForceY1;
  }
  
  gForceX_cal /=1000;
  gForceY_cal /=1000;

  gForceXY_magnitude = sqrt(pow(gForceX_cal,2)+pow(gForceY_cal,2));
  Rz = acos(gForceX_cal/gForceXY_magnitude);
   
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(20,10);
  display.print(gForceX_cal);
  display.print("  ");
  display.print(gForceY_cal);
  display.setCursor(20,25);
  display.print(Rz*180/3.142);
  display.display();
}

