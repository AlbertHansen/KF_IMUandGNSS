// www.moodle.aau.dk/mod/page/view.php?id=1535313
#include <IMUclass.h>
#include <Arduino_LSM6DS3.h>
#include <Arduino.h>
#include <iostream>
using namespace std;



void IMUsetup(){
  if (!IMU.begin()) 
    {
      Serial.println("Failed to initialize IMU! Halting.");
      while (1);
    }
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyro sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
}



int IMUreader::AddMeasurement(float AccX, float AccY, float AccZ, float GyroX, float GyroY, float GyroZ){
  Measurements.at(MeasurementIndex).AccData.AccX = AccX;
  Measurements.at(MeasurementIndex).AccData.AccY = AccY;
  Measurements.at(MeasurementIndex).AccData.AccZ = AccZ;
  Measurements.at(MeasurementIndex).GyroData.pitch = GyroX;
  Measurements.at(MeasurementIndex).GyroData.roll = GyroY;
  Measurements.at(MeasurementIndex).GyroData.yaw = GyroZ;
  return((MeasurementIndex < IMUREADER_OPTIONS.Windowsize - 1) ? MeasurementIndex++ : MeasurementIndex = 0); 
    //Add an indicator of overflow and return true if index is more than N-1
}


void IMUreader::GetMeasurement(){
  float AccX { 0 };
  float AccY { 0 };
  float AccZ { 0 };
  float GyroX { 0 };
  float GyroY { 0 };
  float GyroZ { 0 };

  if (IMU.accelerationAvailable()) 
  {
    IMU.readAcceleration(AccX, AccY, AccZ);
  }
  if (IMU.gyroscopeAvailable()) 
  {
    IMU.readGyroscope(GyroX, GyroY, GyroZ);
  }
  IMUreader::AddMeasurement(AccX, AccY, AccZ, GyroX, GyroY, GyroZ);
  Serial.print("AccX: ");
}


