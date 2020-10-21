#include <M5Stack.h>

#include "gimp_image.h"
#include "surface00.h"

#include "utility/MPU9250.h"
MPU9250 IMU;

//#include <M5StickC.h>

//#include <M5StickCPlus.h>
//#include <SPIFFS.h> 

#include <LovyanGFX.hpp>
#include <Preferences.h>
#include "MadgwickAHRS.h"

Preferences preferences;
Madgwick *filter = new Madgwick();

static LGFX lcd;
static LGFX_Sprite sprite[2];
static LGFX_Sprite sprite_surface[6];

//#pragma GCC optimize ("O3")
struct point3df{ float x, y, z;};
struct surface{ uint8_t p[4]; int16_t z;};
#define U  135         // size of cube
 
struct point3df cubef[8] ={ // cube edge length is 2*U
  { -U, -U,  U },
  {  U, -U,  U },
  {  U, -U, -U },
  { -U, -U, -U },
  { -U,  U,  U },
  {  U,  U,  U },
  {  U,  U, -U },
  { -U,  U, -U },
};
 
struct surface s[6] = {// define the surfaces
  { {0, 1, 2, 3}, 0 }, // bottom
  { {4, 5, 6, 7}, 0 }, // top
  { {0, 1, 5, 4}, 0 }, // back
  { {3, 7, 6, 2}, 0 }, // front
  { {1, 2, 6, 5}, 0 }, // right
  { {0, 3, 7, 4}, 0 }, // left
};
 
struct point3df cubef2[8];

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float offset_pitch = 0.0F;
float offset_roll  = 0.0F;
float offset_yaw   = 0.0F;

double sum_accX = 0.0;
double sum_accY = 0.0;
double sum_accZ = 0.0;

double sum_gyroX = 0.0;
double sum_gyroY = 0.0;
double sum_gyroZ = 0.0;

float calb_accX = 0.0;
float calb_accY = 0.0;
float calb_accZ = 0.0;

float calb_gyroX = 0.0;
float calb_gyroY = 0.0;
float calb_gyroZ = 0.0;

float magX_min = 0.0;
float magY_min = 0.0;
float magZ_min = 0.0;

float magX_max = 0.0;
float magY_max = 0.0;
float magZ_max = 0.0;

bool flip;
uint32_t pre_calc_time = 0;
uint32_t pre_show_time = 0;
unsigned int pre_time = 0;

int ws;
int hs;

void rotate_cube_xyz( float roll, float pitch, float yaw){
  uint8_t i;

  float DistanceCamera = 300;
  float DistanceScreen = 550;

  float cosyaw   = cos(yaw);
  float sinyaw   = sin(yaw);
  float cospitch = cos(pitch);
  float sinpitch = sin(pitch);
  float cosroll  = cos(roll);
  float sinroll  = sin(roll);

  float sinyaw_sinroll = sinyaw * sinroll;
  float sinyaw_cosroll = sinyaw * cosroll;
  float cosyaw_sinroll = cosyaw * sinroll;
  float cosyaw_cosroll = cosyaw * cosroll;

  float x_x = cosyaw * cospitch;
  float x_y = cosyaw_sinroll * sinpitch - sinyaw_cosroll;
  float x_z = cosyaw_cosroll * sinpitch + sinyaw_sinroll;

  float y_x = sinyaw * cospitch;
  float y_y = sinyaw_sinroll * sinpitch + cosyaw_cosroll;
  float y_z = sinyaw_cosroll * sinpitch - cosyaw_sinroll;

  float z_x = -sinpitch;
  float z_y = cospitch * sinroll;
  float z_z = cospitch * cosroll;

  for (i = 0; i < 8; i++){
    float x = x_x * cubef[i].x
            + x_y * cubef[i].y
            + x_z * cubef[i].z;
    float y = y_x * cubef[i].x
            + y_y * cubef[i].y
            + y_z * cubef[i].z;
    float z = z_x * cubef[i].x
            + z_y * cubef[i].y
            + z_z * cubef[i].z;

    cubef2[i].x = (x * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (ws>>1);
    cubef2[i].y = (y * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (hs>>1);
    cubef2[i].z = z;
  }
}


void rotate_cube_quaternion(float a, float b, float c, float d)
{
  uint8_t i;

  float DistanceCamera = 300;
  float DistanceScreen = 550;

  float x_x = a*a + b*b - c*c - d*d;
  float x_y = 2*b*c + 2*a*d;
  float x_z = 2*b*d - 2*a*c;
  float y_x = 2*b*c - 2*a*d;
  float y_y = a*a - b*b + c*c - d*d;
  float y_z = 2*c*d + 2*a*b;
  float z_x = 2*b*d + 2*a*c;
  float z_y = 2*c*d - 2*a*b;
  float z_z = a*a - b*b - c*c + d*d;

  for (i = 0; i < 8; i++){
    float x = x_x * cubef[i].x
            + x_y * cubef[i].y
            + x_z * cubef[i].z;
    float y = y_x * cubef[i].x
            + y_y * cubef[i].y
            + y_z * cubef[i].z;
    float z = z_x * cubef[i].x
            + z_y * cubef[i].y
            + z_z * cubef[i].z;

    cubef2[i].x = (x * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (ws>>1);
    cubef2[i].y = (y * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (hs>>1);
    cubef2[i].z = z;
  }
}

void setup(void){ 
  M5.begin();
  Wire.begin();
  //M5.IMU.Init();
  
  preferences.begin("imu_calb_data", true);
//  calb_accX = preferences.getFloat("ax", 0);
//  calb_accY = preferences.getFloat("ay", 0);
//  calb_accZ = preferences.getFloat("az", 0);
//  calb_gyroX = preferences.getFloat("gx", 0);
//  calb_gyroY = preferences.getFloat("gy", 0);
//  calb_gyroZ = preferences.getFloat("gz", 0);
  
  magX_min = preferences.getFloat("mx_min", 0);
  magY_min = preferences.getFloat("my_min", 0);
  magZ_min = preferences.getFloat("mz_min", 0);
  magX_max = preferences.getFloat("mx_max", 0);
  magY_max = preferences.getFloat("my_max", 0);
  magZ_max = preferences.getFloat("mz_max", 0);
  
  Serial.println(calb_accX);
  Serial.println(calb_accY);
  Serial.println(calb_accZ);
  Serial.println(calb_gyroX);
  Serial.println(calb_gyroY);
  Serial.println(calb_gyroZ);
  
  Serial.println(magX_min);
  Serial.println(magY_min);
  Serial.println(magZ_min);
  Serial.println(magX_max);
  Serial.println(magY_max);
  Serial.println(magZ_max);
  preferences.end();

  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();
  IMU.initAK8963(IMU.magCalibration);

  lcd.init();

  lcd.setRotation(1);


// バックライトの輝度を 0～255 の範囲で設定します。
//  lcd.setBrightness(80);

  lcd.setColorDepth(16);  // RGB565の16ビットに設定

  lcd.fillScreen(0);

  //ws = lcd.width();
  //hs = lcd.height();
  ws = 160;
  hs = 160;

  sprite[0].createSprite(ws,hs);
  sprite[1].createSprite(ws,hs);

  sprite_surface[0].createSprite(surface00.width ,surface00.height);
  sprite_surface[0].pushImage(  0, 0,surface00.width ,surface00.height , (lgfx:: rgb565_t*)surface00.pixel_data);
  
  lcd.startWrite();
  lcd.fillScreen(TFT_DARKGREY);
}

float smoothMove(float dst, float src)
{
  if (     dst + M_PI < src) src -= M_PI * 2;
  else if (src + M_PI < dst) src += M_PI * 2;
  return (dst + src * 19.0) / 20.0;
}

void loop(void)
{
  int calc_time = millis() - pre_calc_time;
  
  //count = (count + 1) & 3;
  //if (count == 0)
  //if (calc_time >= 9)
  if (getIMUData(true))
  {  
    M5.update();
    if(M5.BtnA.isPressed()){
      Serial.println("Calibration Start");
      getCalibrationVal();
    }else if(M5.BtnB.isPressed()){
      Serial.println("Mag Calibration Start");
      getMagCalibrationVal();
    }else if(M5.BtnC.isPressed()){
      //Serial.println("Mag Calibration Start");
      //getMagCalibrationVal();
      offset_pitch = filter->getPitch();
      offset_roll  = filter->getRoll();
      offset_yaw   = filter->getYaw();
    }

//    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
//    M5.IMU.getAccelData(&accX,&accY,&accZ);
//
//    filter->MadgwickAHRSupdateIMU(
//      PI/180.0F*(gyroX - calb_gyroX), 
//      PI/180.0F*(gyroY - calb_gyroY), 
//      PI/180.0F*(gyroZ - calb_gyroZ),
//      accX - calb_accX, 
//      accY - calb_accY, 
//      accZ - calb_accZ
//      );

    pre_calc_time = millis();
  }

  //pitch = smoothMove(filter->getPitch(), pitch);
  //roll  = smoothMove(filter->getRoll() , roll );
  //yaw   = smoothMove(filter->getYaw()  , yaw  );

  pitch = filter->getPitch()-offset_pitch;
  roll  = filter->getRoll()-offset_roll;
  yaw   = filter->getYaw()-offset_yaw;

  //rotate_cube_xyz(roll,pitch,yaw);
  rotate_cube_quaternion(filter->q0, filter->q1, filter->q2, filter->q3);

  //描写する面の順番に並び替え
//  int ss[6]={0,1,2,3,4,5};
//  float sf[6]={0};
//  for (int i = 0; i < 6; i++)
//  {
//    float wz = 0;
//    for(int j=0;j<4;j++){
//      wz += cubef2[s[i].p[j]].z;
//    }
//    sf[i] = wz;
//  }
//  //交換ソート
//  for (int j = 5; j > 0; j--){
//    for (int i = 0; i < j; i++)
//    {
//        if(sf[i] < sf[i+1])
//        {
//          float work = sf[i];
//          sf[i] = sf[i+1];
//          sf[i+1] = work;
//          
//          int iw = ss[i];
//          ss[i] = ss[i+1];
//          ss[i+1] = iw;
//        }
//    }
//  }

  flip = !flip;
  sprite[flip].clear();
  for (int i = 0; i < 8; i++)
  {
    sprite[flip].drawRect( (int)cubef2[i].x-2, (int)cubef2[i].y-2, 4, 4 , 0xF000);
    //Serial.printf("%d,%f,%f,\r\n",i,cubef2[i].x, cubef2[i].y); 
  }

//  for (int i = 0; i < 6; i++)
//  {
//    int ii = ss[i];
//    
//    sprite[flip].setColor( lcd.color565(((( ii + 1) &  1)      * 255),
//                                  ((((ii + 1) >> 1) & 1) * 255),
//                                  ((((ii + 1) >> 2) & 1) * 255)
//                   )             );
//    sprite[flip].fillTriangle(    cubef2[s[ii].p[0]].x, cubef2[s[ii].p[0]].y,
//                            cubef2[s[ii].p[1]].x, cubef2[s[ii].p[1]].y,
//                            cubef2[s[ii].p[2]].x, cubef2[s[ii].p[2]].y);
//    sprite[flip].fillTriangle(    cubef2[s[ii].p[2]].x, cubef2[s[ii].p[2]].y,
//                            cubef2[s[ii].p[3]].x, cubef2[s[ii].p[3]].y,
//                            cubef2[s[ii].p[0]].x, cubef2[s[ii].p[0]].y);
//  }
  
  int show_time = millis() - pre_show_time;
  pre_show_time = millis();
  sprite[flip].setCursor(0, 10);
  sprite[flip].printf("%+3.2f",pitch);
  sprite[flip].setCursor(0, 20);
  sprite[flip].printf("%+3.2f",roll);
  sprite[flip].setCursor(0, 30);
  sprite[flip].printf("%+3.2f",yaw);
  
  sprite[flip].setCursor(0, 50);
  sprite[flip].printf("%5d",show_time);

  if (calc_time >= 9){
    sprite[flip].setCursor(0, 60);
    sprite[flip].printf("%5d",calc_time);
  }
  
  sprite[flip].pushSprite(&lcd, 0, 0);

  
  float x_zoom = (90-abs(pitch*180/PI))/90;
  float y_zoom = (90-abs(roll*180/PI))/90;


  {
    int ii = 3; //front
    
    float surface_center_x = 0.0;
    float surface_center_y = 0.0;
    
    for(int i=0 ; i<4 ; i++){
      surface_center_x += cubef2[s[ii].p[i]].x;
      surface_center_y += cubef2[s[ii].p[i]].y;
      //Serial.printf("%f,%f,\r\n",cubef2[s[ii].p[i]].x, cubef2[s[ii].p[i]].y); 
    }
    
    sprite_surface[0].pushRotateZoom(&lcd, (int)(surface_center_x/4.0), (int)(surface_center_y/4.0), -yaw*180/PI, x_zoom, y_zoom);
  }

}

bool getIMUData(bool calc_flag)
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres();

    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;

    IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
    IMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    //IMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    //IMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    //IMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] -
             IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] -
             IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] -
             IMU.magbias[2];

    IMU.tempCount = IMU.readTempData();  // Read the adc values
    // Temperature in degrees Centigrade
    IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;

//    filter->MadgwickAHRSupdateIMU(
//      PI/180.0F*(IMU.gx - calb_gyroX), 
//      PI/180.0F*(IMU.gy - calb_gyroY), 
//      PI/180.0F*(IMU.gz - calb_gyroZ),
//      IMU.ax - calb_accX, 
//      IMU.ay - calb_accY, 
//      IMU.az - calb_accZ
//    );

    if(calc_flag){
      filter->MadgwickAHRSupdate(
        PI/180.0F*(IMU.gx - calb_gyroX), 
        PI/180.0F*(IMU.gy - calb_gyroY), 
        PI/180.0F*(IMU.gz - calb_gyroZ),
        IMU.ax - calb_accX, 
        IMU.ay - calb_accY, 
        IMU.az - calb_accZ,
        (2.0*IMU.my-magY_max-magY_min)/(magY_max-magY_min),
        (2.0*IMU.mx-magX_max-magX_min)/(magX_max-magX_min),
        -(2.0*IMU.mz-magZ_max-magZ_min)/(magZ_max-magZ_min)
      );
    }
  
    //pitch = filter->getPitch()*180.0F/PI;
    //roll = filter->getRoll()*180.0F/PI;
    //yaw = filter->getYaw()*180.0F/PI;

    return true;
  }

  return false;
}

void getCalibrationVal()
{
    const int CalbNum = 1000;
    sum_accX = 0.0;
    sum_accY = 0.0;
    sum_accZ = 0.0;

    sum_gyroX = 0.0;
    sum_gyroY = 0.0;
    sum_gyroZ = 0.0;  
    
    for(int i = 4 ; i >= 0 ; i--){
      lcd.setCursor(0, 70);
      lcd.printf("Start Calb %d sec",i);
      delay(1000);
    }

    for(int i = 0 ; i < CalbNum ; i++){

      if(getIMUData(false)){
        sum_accX += IMU.ax;
        sum_accY += IMU.ay;
        sum_accZ += IMU.az;

        sum_gyroX += IMU.gx;
        sum_gyroY += IMU.gy;
        sum_gyroZ += IMU.gz;
        
        int deftime = millis() - pre_time;
        while(deftime < 10)
        {
          delay(1);
          deftime = millis() - pre_time;
        }
        lcd.setCursor(0, 70); lcd.printf("%5d",deftime);
        pre_time = millis();

        lcd.setCursor(0, 70);
        lcd.printf("Calb count : %3d  ",CalbNum-i);
      }
    }

    calb_accX = (float)(sum_accX/CalbNum);
    calb_accY = (float)(sum_accY/CalbNum);
    calb_accZ = (float)(sum_accZ/CalbNum) - 1.0F;

    calb_gyroX = (float)(sum_gyroX/CalbNum);
    calb_gyroY = (float)(sum_gyroY/CalbNum);
    calb_gyroZ = (float)(sum_gyroZ/CalbNum);

    preferences.begin("imu_calb_data", false);
    preferences.putFloat("ax", calb_accX);
    preferences.putFloat("ay", calb_accY);
    preferences.putFloat("az", calb_accX);
    preferences.putFloat("gx", calb_gyroX);
    preferences.putFloat("gy", calb_gyroY);
    preferences.putFloat("gz", calb_gyroZ);
    preferences.end();
    
    lcd.setCursor(0, 70);
    lcd.printf("End Calb");
}

#define FLT_MIN -3.4028234663852886e+38
#define FLT_MAX  3.4028234663852886e+38

void getMagCalibrationVal()
{
  const int CalbNum = 3000;
  magX_min = FLT_MAX;
  magY_min = FLT_MAX;
  magZ_min = FLT_MAX;

  magX_max = FLT_MIN;
  magY_max = FLT_MIN;
  magZ_max = FLT_MIN;  
    
  for(int i = 4 ; i >= 0 ; i--){
    lcd.setCursor(0, 70);
    lcd.printf("Start Mag Calb %d sec  ",i);
    delay(1000);
  }
    
  for(int i = 0 ; i < CalbNum ; i++){

      if(getIMUData(false)){

        if(magX_min > IMU.mx){
          magX_min = IMU.mx;
        }
        
        if(magY_min > IMU.my){
          magY_min = IMU.my;
        }
        
        if(magZ_min > IMU.mz){
          magZ_min = IMU.mz;
        }

        if(magX_max < IMU.mx){
          magX_max = IMU.mx;
        }
        
        if(magY_max < IMU.my){
          magY_max = IMU.my;
        }
        
        if(magZ_max < IMU.mz){
          magZ_max = IMU.mz;
        }

        int deftime = millis() - pre_time;
        while(deftime < 10)
        {
          delay(1);
          deftime = millis() - pre_time;
        }
        lcd.setCursor(0, 60); lcd.printf("%5d",deftime);
        pre_time = millis();

        lcd.setCursor(0, 70);
        lcd.printf("Calb count : %3d  ",CalbNum-i);
      }
  }

  preferences.begin("imu_calb_data", false);
  preferences.putFloat("mx_min", magX_min);
  preferences.putFloat("my_min", magY_min);
  preferences.putFloat("mz_min", magZ_min);
  preferences.putFloat("mx_max", magX_max);
  preferences.putFloat("my_max", magY_max);
  preferences.putFloat("mz_max", magZ_max);
  preferences.end();
  
  lcd.setCursor(0, 70);
  lcd.printf("End Calb        ");
}
