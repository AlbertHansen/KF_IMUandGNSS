#include <IMUReader.h>


void IMUReader::AddMeasurement(size_t AccX, size_t AccY, size_t AccZ, size_t GyroX, size_t GyroY, size_t GyroZ)
{
    Measurements.at(MeasurementIndex).AccData.x = AccX;
    Measurements.at(MeasurementIndex).AccData.y = AccY;
    Measurements.at(MeasurementIndex).AccData.z = AccZ;
    Measurements.at(MeasurementIndex).GyroData.pitch = GyroX;
    Measurements.at(MeasurementIndex).GyroData.roll = GyroY;
    Measurements.at(MeasurementIndex).GyroData.yaw = GyroZ;

    if (MeasurementCount < IMUREADER_OPTIONS.Windowsize)
    {
        MeasurementCount++;
    }
}

void IMUReader::GetAccMean() const
{

}

void IMUReader::GetGyroMean() const
{

}
