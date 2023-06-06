// www.moodle.aau.dk/mod/page/view.php?id=1535313
#include "IMUclass.h"

void IMUreader::MakeMeasurements()
{
  std::vector<std::vector<float>> currentMeasurement {
      { 0 },  // acc x measurement
      { 0 },  // acc y measurement
      { 0 },  // acc z measurement
      { 0 }, // omega x measurement
      { 0 }, // omega y measurement
      { 0 }}; // omega z measurement
    
  std::vector<std::vector<float>> angle { 
      {Direction.x}, 
      {Direction.y}, 
      {Direction.z}}; // radians
  
  for(size_t i = 0; i < IMUREADER_OPTIONS.Windowsize; i++)
  { 
    int start_time = millis();
    IMU.readGyroscope(   currentMeasurement.at(3).at(0), currentMeasurement.at(4).at(0), currentMeasurement.at(5).at(0));
    int end_time = millis();

    angle.at(2).at(0) += currentMeasurement.at(5).at(0) * (end_time - start_time) / 1000.f * 3.14159265359f / 180.f;  // radians
    // angle.at(2).at(0) += currentMeasurement.at(5).at(0) * (1) / 1000.f * 3.14159265359f / 180.f;  // radians

    std::vector<std::vector<float>> accelerationData { {0.f}, {0.f}, {0.f} };
    IMU.readAcceleration(accelerationData.at(0).at(0), accelerationData.at(1).at(0), accelerationData.at(2).at(0));
    
    // Correct acceleration angles based on angle.at(2).at(0) (rotation around z-axis)
    currentMeasurement.at(0).at(0) = std::cos(angle.at(2).at(0)) * accelerationData.at(0).at(0) + std::sin(angle.at(2).at(0)) * accelerationData.at(1).at(0);
    currentMeasurement.at(1).at(0) = std::cos(angle.at(2).at(0)) * accelerationData.at(1).at(0) + std::sin(angle.at(2).at(0)) * accelerationData.at(0).at(0);

    AddMeasurement(currentMeasurement);
  }
  Direction.x = angle.at(0).at(0);
  Direction.y = angle.at(1).at(0);
  Direction.z = angle.at(2).at(0);
};

void IMUreader::AddMeasurement(const std::vector<std::vector<float>>& AccGyroMeasurement){

  Measurements.at(MeasurementIndex).AccData.AccX    = AccGyroMeasurement.at(0).at(0);
  Measurements.at(MeasurementIndex).AccData.AccY    = AccGyroMeasurement.at(1).at(0);
  Measurements.at(MeasurementIndex).AccData.AccZ    = AccGyroMeasurement.at(2).at(0);
  Measurements.at(MeasurementIndex).GyroData.pitch  = AccGyroMeasurement.at(3).at(0);
  Measurements.at(MeasurementIndex).GyroData.roll   = AccGyroMeasurement.at(4).at(0);
  Measurements.at(MeasurementIndex).GyroData.yaw    = AccGyroMeasurement.at(5).at(0);

  MeasurementIndex++;
  if (MeasurementIndex > IMUREADER_OPTIONS.Windowsize - 1)
  {
    MeasurementIndex = 0;
    // Serial.println("MeasurementIndex reset to 0");
  }
};


std::vector<std::vector<float>> IMUreader::GetAccMean() const
{
  // Serial.println("GetAccMean");
  std::vector<std::vector<float>> AccMean { 
      {0.f}, 
      {0.f}, 
      {0.f}};

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
  // Serial.println("GetGyroMean");
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
float IMUreader::GetYaw() const
{
  return Direction.z;
};