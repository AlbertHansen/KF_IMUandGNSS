#ifndef IMUREADER_H_
#define IMUREADER_H_

#include <Arduino.h>
#include <tuple>
#include <array>

constexpr struct OPTIONS
{
    int Windowsize = 200;
} IMUREADER_OPTIONS;

class IMUReader
{
    public:
        void AddMeasurement(size_t AccX, size_t AccY, size_t AccZ, size_t GyroX, size_t GyroY, size_t GyroZ);
        void GetAccMean() const;
        void GetGyroMean() const;

        class Measurement
        {
            public:
                struct AccelerationData
                {
                    float x { 0 };
                    float y { 0 };
                    float z { 0 };
                } AccData;

                struct GyroData
                {
                    float pitch { 0 };
                    float roll  { 0 };
                    float yaw   { 0 };
                } GyroData;
                
            private:
        };

    private:
        std::array<Measurement, IMUREADER_OPTIONS.Windowsize> Measurements;
        int MeasurementIndex { 0 };
        int MeasurementCount { 0 };
};

#endif