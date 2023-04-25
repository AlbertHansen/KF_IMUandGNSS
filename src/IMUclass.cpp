// www.moodle.aau.dk/mod/page/view.php?id=1535313
#include <IMUclass.h>
#include <Arduino_LSM6DS3.h>
#include <Arduino.h>
#include <iostream>
using namespace std;




int IMUreader::AddMeasurement(float AccX, float AccY, float AccZ, float GyroX, float GyroY, float GyroZ){
  Serial.println(AccX);
  Serial.println(AccY);
  Serial.println(AccZ);
  Serial.println(GyroX);
  Serial.println(GyroY);
  Serial.println(GyroZ);
  Measurements.at(MeasurementIndex).AccData.AccX = AccX;
  Measurements.at(MeasurementIndex).AccData.AccY = AccY;
  Measurements.at(MeasurementIndex).AccData.AccZ = AccZ;
  Measurements.at(MeasurementIndex).GyroData.pitch = GyroX;
  Measurements.at(MeasurementIndex).GyroData.roll = GyroY;
  Measurements.at(MeasurementIndex).GyroData.yaw = GyroZ;
  if (MeasurementIndex > IMUREADER_OPTIONS.Windowsize - 1)
  {
    MeasurementIndex = 0;
    Serial.println("MeasurementIndex reset to 0");
  }
  else
  {
    MeasurementIndex++;
    return(MeasurementIndex);
  }
}


int IMUreader::GetMeasurement(){
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
  return(AddMeasurement(AccX, AccY, AccZ, GyroX, GyroY, GyroZ));
}

void IMUreader::GetAccMean(){
  Serial.println("GetAccMean");
  float AccXMean { 0 };
  float AccYMean { 0 };
  float AccZMean { 0 };
  for (int i = 0; i < IMUREADER_OPTIONS.Windowsize; i++)
  {
    AccXMean += Measurements.at(i).AccData.AccX;
    AccYMean += Measurements.at(i).AccData.AccY;
    AccZMean += Measurements.at(i).AccData.AccZ;
  }
  AccXMean /= IMUREADER_OPTIONS.Windowsize;
  AccYMean /= IMUREADER_OPTIONS.Windowsize;
  AccZMean /= IMUREADER_OPTIONS.Windowsize;

  Serial.println(AccXMean);
  Serial.println(AccYMean);
  Serial.println(AccZMean);

}


void IMUreader::GetGyroMean(){
  float GyroXMean { 0 };
  float GyroYMean { 0 };
  float GyroZMean { 0 };
  for (int i = 0; i < IMUREADER_OPTIONS.Windowsize; i++)
  {
    GyroXMean += Measurements.at(i).GyroData.pitch;
    GyroYMean += Measurements.at(i).GyroData.roll;
    GyroZMean += Measurements.at(i).GyroData.yaw;
  }
  GyroXMean /= IMUREADER_OPTIONS.Windowsize;
  GyroYMean /= IMUREADER_OPTIONS.Windowsize;
  GyroZMean /= IMUREADER_OPTIONS.Windowsize;


}
