// www.moodle.aau.dk/mod/page/view.php?id=1535313
#include "IMUclass.h"

void IMUreader::AddMeasurement(const std::vector<std::vector<float>>& AccGyroMeasurement){

  Measurements.at(MeasurementIndex).AccData.AccX    = AccGyroMeasurement.at(0).at(0);
  Measurements.at(MeasurementIndex).AccData.AccY    = AccGyroMeasurement.at(1).at(0);
  Measurements.at(MeasurementIndex).AccData.AccZ    = AccGyroMeasurement.at(2).at(0);
  Measurements.at(MeasurementIndex).GyroData.pitch  = AccGyroMeasurement.at(3).at(0);
  Measurements.at(MeasurementIndex).GyroData.roll   = AccGyroMeasurement.at(4).at(0);
  Measurements.at(MeasurementIndex).GyroData.yaw    = AccGyroMeasurement.at(5).at(0);

  MeasurementIndex++;
  if (MeasurementIndex > IMUREADER_OPTIONS.Windowsize)
  {
    MeasurementIndex = 0;
    Serial.println("MeasurementIndex reset to 0");
  }
};


std::vector<std::vector<float>> IMUreader::GetAccMean() const
{
  Serial.println("GetAccMean");
  std::vector<std::vector<float>> AccMean { {0.f}, {0.f}, {0.f} };
  for (int i = 0; i < IMUREADER_OPTIONS.Windowsize; i++)
  {
    AccMean.at(0).at(0) += Measurements.at(i).AccData.AccX;
    AccMean.at(1).at(0) += Measurements.at(i).AccData.AccY;
    AccMean.at(2).at(0) += Measurements.at(i).AccData.AccZ;
  }
  AccMean.at(0).at(0) /= IMUREADER_OPTIONS.Windowsize;
  AccMean.at(1).at(0) /= IMUREADER_OPTIONS.Windowsize;
  AccMean.at(2).at(0) /= IMUREADER_OPTIONS.Windowsize;
  return AccMean;
};


std::vector<std::vector<float>> IMUreader::GetGyroMean() const
{
  Serial.println("GetGyroMean");
  std::vector<std::vector<float>> GyroMean { {0.f}, {0.f}, {0.f} };
  for (int i = 0; i < IMUREADER_OPTIONS.Windowsize; i++)
  {
    GyroMean.at(0).at(0) += Measurements.at(i).GyroData.pitch;
    GyroMean.at(1).at(0) += Measurements.at(i).GyroData.roll;
    GyroMean.at(2).at(0) += Measurements.at(i).GyroData.yaw;
  }
  GyroMean.at(0).at(0) /= IMUREADER_OPTIONS.Windowsize;
  GyroMean.at(1).at(0) /= IMUREADER_OPTIONS.Windowsize;
  GyroMean.at(2).at(0) /= IMUREADER_OPTIONS.Windowsize;
  
  return GyroMean;
};
